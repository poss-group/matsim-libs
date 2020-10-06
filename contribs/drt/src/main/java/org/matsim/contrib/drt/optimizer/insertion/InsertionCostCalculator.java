/* *********************************************************************** *
 * project: org.matsim.*
 *                                                                         *
 * *********************************************************************** *
 *                                                                         *
 * copyright       : (C) 2017 by the members listed in the COPYING,        *
 *                   LICENSE and WARRANTY file.                            *
 * email           : info at matsim dot org                                *
 *                                                                         *
 * *********************************************************************** *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *   See also COPYING, LICENSE and WARRANTY file                           *
 *                                                                         *
 * *********************************************************************** */

package org.matsim.contrib.drt.optimizer.insertion;

import static org.matsim.contrib.drt.schedule.DrtTaskBaseType.STOP;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.ToDoubleFunction;
import java.util.stream.IntStream;

import org.apache.log4j.Logger;
import org.matsim.api.core.v01.network.Link;
import org.matsim.contrib.drt.optimizer.VehicleData;
import org.matsim.contrib.drt.optimizer.VehicleData.Stop;
import org.matsim.contrib.drt.passenger.DrtRequest;
import org.matsim.contrib.drt.passenger.DrtRequestCreator;
import org.matsim.contrib.drt.routing.DefaultDrtRouteUpdater;
import org.matsim.contrib.drt.routing.DrtRoute;
import org.matsim.contrib.drt.routing.DrtRouteCreator;
import org.matsim.contrib.drt.routing.DrtRouteFactory;
import org.matsim.contrib.drt.run.DrtConfigGroup;
import org.matsim.contrib.drt.schedule.DrtStayTask;
import org.matsim.contrib.drt.run.DrtModeModule.DrtRouteCreatorProvider;
import org.matsim.contrib.dvrp.schedule.Schedule;
import org.matsim.contrib.dvrp.schedule.Schedule.ScheduleStatus;
import org.matsim.contrib.dvrp.schedule.Schedules;
import org.matsim.core.mobsim.framework.MobsimTimer;
import org.matsim.contrib.util.distance.DistanceUtils;
import org.matsim.core.population.routes.RouteFactories;

/**
 * @author michalm
 */
public class InsertionCostCalculator<D> {
    public interface PenaltyCalculator {
        double calcPenalty(double maxWaitTimeViolation, double maxTravelTimeViolation);
    }

    public static class RejectSoftConstraintViolations implements PenaltyCalculator {
        @Override
        public double calcPenalty(double maxWaitTimeViolation, double maxTravelTimeViolation) {
            return maxWaitTimeViolation > 0 || maxTravelTimeViolation > 0 ? INFEASIBLE_SOLUTION_COST : 0;
        }
    }

    public static class DiscourageSoftConstraintViolations implements PenaltyCalculator {
        //XXX try to keep penalties reasonably high to prevent people waiting or travelling for hours
        //XXX however, at the same time prefer max-wait-time to max-travel-time violations
        private static final double MAX_WAIT_TIME_VIOLATION_PENALTY = 1;// 1 second of penalty per 1 second of late
		// departure
        private static final double MAX_TRAVEL_TIME_VIOLATION_PENALTY = 10;// 10 seconds of penalty per 1 second of
		// late arrival

        @Override
        public double calcPenalty(double maxWaitTimeViolation, double maxTravelTimeViolation) {
            return MAX_WAIT_TIME_VIOLATION_PENALTY * maxWaitTimeViolation
                    + MAX_TRAVEL_TIME_VIOLATION_PENALTY * maxTravelTimeViolation;
        }
    }

    public static final double INFEASIBLE_SOLUTION_COST = Double.MAX_VALUE / 2;

    private final double stopDuration;
    private final DoubleSupplier timeOfDay;
    private final PenaltyCalculator penaltyCalculator;
    private final ToDoubleFunction<D> detourTime;

    private final static Logger LOG = Logger.getLogger(InsertionCostCalculator.class.getName());
    private final static double DETOUR_DELTA = 999;//1.5;
    private final static double RELATIVE_DELIVERY_DELTA = 3;
    private static DrtRouteCreatorProvider drtRouteCreatorProvider;

    public InsertionCostCalculator(DrtConfigGroup drtConfig, MobsimTimer timer, PenaltyCalculator penaltyCalculator,
                                   ToDoubleFunction<D> detourTime) {
        this(drtConfig.getStopDuration(), timer::getTimeOfDay, penaltyCalculator, detourTime);
    }

    public InsertionCostCalculator(double stopDuration, DoubleSupplier timeOfDay, PenaltyCalculator penaltyCalculator,
                                   ToDoubleFunction<D> detourTime) {
        this.stopDuration = stopDuration;
        this.timeOfDay = timeOfDay;
        this.penaltyCalculator = penaltyCalculator;
        this.detourTime = detourTime;
    }

    public static void setDrtRouteCreatorProvider(DrtRouteCreatorProvider drtRouteCreatorProvider) {
        InsertionCostCalculator.drtRouteCreatorProvider = drtRouteCreatorProvider;
    }

    /**
     * As the main goal is to minimise bus operation time, this method calculates how much longer the bus will operate
     * after insertion. By returning a value equal or higher than INFEASIBLE_SOLUTION_COST, the insertion is considered
     * infeasible
     * <p>
     * The insertion is invalid if some maxTravel/Wait constraints for the already scheduled requests are not fulfilled
     * or the vehicle's time window is violated (hard constraints). This is denoted by returning
     * INFEASIBLE_SOLUTION_COST.
     * <p>
     * However, not fulfilling the maxTravel/Time constraints (soft constraints) is penalised using
     * PenaltyCalculator. If the penalty is at least as high as INFEASIBLE_SOLUTION_COST, the soft
     * constraint becomes effectively a hard one.
     *
     * @param drtRequest the request
     * @param insertion  the insertion to be considered here, with PickupIdx and DropoffIdx the positions
     * @return cost of insertion (values higher or equal to INFEASIBLE_SOLUTION_COST represent an infeasible insertion)
     */
    public double calculate(DrtRequest drtRequest, InsertionWithDetourData<D> insertion) {
        //TODO precompute time slacks for each stop to filter out even more infeasible insertions ???????????

        double[] toFromPickupToFromDropoff = {0, 0, 0, 0};
        double pickupDetourTimeLoss = calculatePickupDetourTimeLoss(drtRequest, insertion, toFromPickupToFromDropoff);
        double dropoffDetourTimeLoss = calculateDropoffDetourTimeLoss(drtRequest, insertion, toFromPickupToFromDropoff);
        // the pickupTimeLoss is needed for stops that suffer only that one, while the sum of both will be suffered by
        // the stops after the dropoff stop. kai, nov'18
        // The computation is complicated; presumably, it takes care of this.  kai, nov'18

        // this is what we want to minimise
        double totalTimeLoss = pickupDetourTimeLoss + dropoffDetourTimeLoss;
        if (isHardConstraintsViolated(insertion, pickupDetourTimeLoss, totalTimeLoss, drtRequest,
                toFromPickupToFromDropoff)) {
            return INFEASIBLE_SOLUTION_COST;
        }

        return totalTimeLoss + calcSoftConstraintPenalty(drtRequest, insertion, pickupDetourTimeLoss);
    }

    double calculatePickupDetourTimeLoss(DrtRequest drtRequest, InsertionWithDetourData<D> insertion,
                                         double[] toFromPickupToFromDropoff) {
        VehicleData.Entry vEntry = insertion.getVehicleEntry();
        final int pickupIdx = insertion.getPickup().index;
        final int dropoffIdx = insertion.getDropoff().index;

        // 'no detour' is also possible now for pickupIdx==0 if the currentTask is STOP
        Schedule schedule = vEntry.vehicle.getSchedule();
        boolean ongoingStopTask = pickupIdx == 0 && schedule.getStatus() == ScheduleStatus.STARTED//
                && STOP.isBaseTypeOf(schedule.getCurrentTask());

        Link pickupPreviousLink = insertion.getPickup().previousLink;

        Link pickupNextLink = insertion.getPickup().nextLink;
        Link insertedLink = drtRequest.getFromLink();
//        (insertion.getVehicleEntry().stops.size() > 1) &&
        if (isEllipseConstraintViolated(pickupPreviousLink, insertedLink,
                pickupNextLink)) { // pick-up-ellipse constraint
//            LOG.debug("pick-up-ellipse is violated");
            return INFEASIBLE_SOLUTION_COST;
        }

        if ((ongoingStopTask && drtRequest.getFromLink() == pickupPreviousLink) //
                || (pickupIdx > 0 && drtRequest.getFromLink() == pickupPreviousLink)) {
            if (pickupIdx != dropoffIdx) {// not: PICKUP->DROPOFF
                return 0;// no detour
            }

            // PICKUP->DROPOFF
            // no extra drive to pickup and stop (==> toPickupTT == 0 and stopDuration == 0)
            double fromPickupTT = detourTime.applyAsDouble(insertion.getDetourFromPickup());
			toFromPickupToFromDropoff[1] = fromPickupTT;
			double replacedDriveTT = calculateReplacedDriveDuration(vEntry, pickupIdx);
            return fromPickupTT - replacedDriveTT;
        }

        double toPickupTT = detourTime.applyAsDouble(insertion.getDetourToPickup());
        double fromPickupTT = detourTime.applyAsDouble(insertion.getDetourFromPickup());
		toFromPickupToFromDropoff[0] = toPickupTT;
		toFromPickupToFromDropoff[1] = fromPickupTT;
		double replacedDriveTT = pickupIdx == dropoffIdx // PICKUP->DROPOFF ?
                ? 0 // no drive following the pickup is replaced (only the one following the dropoff)
                : calculateReplacedDriveDuration(vEntry, pickupIdx);
        return toPickupTT + stopDuration + fromPickupTT - replacedDriveTT;
    }

    double calculateDropoffDetourTimeLoss(DrtRequest drtRequest, InsertionWithDetourData<D> insertion,
                                          double[] toFromPickupToFromDropoff) {
        VehicleData.Entry vEntry = insertion.getVehicleEntry();
        final int pickupIdx = insertion.getPickup().index;
        final int dropoffIdx = insertion.getDropoff().index;


		Link dropoffPreviousLink = insertion.getPickup().previousLink;
		Link dropoffNextLink = insertion.getPickup().nextLink;
		Link insertedLink = drtRequest.getFromLink();
		if (isEllipseConstraintViolated(dropoffPreviousLink, insertedLink,
				dropoffNextLink)) { // drop-off-ellipse constraint
//            LOG.debug("drop-off-ellipse is violated");
			return INFEASIBLE_SOLUTION_COST;
		}

        if (dropoffIdx > 0 && pickupIdx != dropoffIdx && drtRequest.getToLink() == vEntry.stops.get(dropoffIdx - 1).task
                .getLink()) {
            return 0; // no detour
        }

        double toDropoffTT = dropoffIdx == pickupIdx // PICKUP->DROPOFF ?
                ? 0 // PICKUP->DROPOFF taken into account as fromPickupTT
                : detourTime.applyAsDouble(insertion.getDetourToDropoff());
        double fromDropoffTT = dropoffIdx == vEntry.stops.size() // DROPOFF->STAY ?
                ? 0 //
                : detourTime.applyAsDouble(insertion.getDetourFromDropoff());
        toFromPickupToFromDropoff[2] = toDropoffTT;
        toFromPickupToFromDropoff[3] = fromDropoffTT;
        double replacedDriveTT = dropoffIdx == pickupIdx // PICKUP->DROPOFF ?
                ? 0 // replacedDriveTT already taken into account in pickupDetourTimeLoss
                : calculateReplacedDriveDuration(vEntry, dropoffIdx);
        return toDropoffTT + stopDuration + fromDropoffTT - replacedDriveTT;
    }

    private double calculateReplacedDriveDuration(VehicleData.Entry vEntry, int insertionIdx) {
        if (insertionIdx == vEntry.stops.size()) {
            return 0;// end of route - bus would wait there
        }

        double replacedDriveStartTime = getDriveToInsertionStartTime(vEntry, insertionIdx);
        double replacedDriveEndTime = vEntry.stops.get(insertionIdx).task.getBeginTime();
        return replacedDriveEndTime - replacedDriveStartTime;
    }

    private boolean isHardConstraintsViolated(InsertionWithDetourData<?> insertion, double pickupDetourTimeLoss,
                                              double totalTimeLoss,
                                              DrtRequest drtRequest, double[] toFromPickupToFromDropoff) {
        VehicleData.Entry vEntry = insertion.getVehicleEntry();
        final int pickupIdx = insertion.getPickup().index;
        final int dropoffIdx = insertion.getDropoff().index;

        // calculating the travel time for the current passenger
//        double detourTravelTime = toFromPickupToFromDropoff[1] + toFromPickupToFromDropoff[2];
//        double lastStopDepartureTime = pickupIdx != dropoffIdx ? vEntry.stops.get(pickupIdx).getDepartureTime() : 0;
        List<Link> stopList = new ArrayList<>();
        stopList.add(drtRequest.getFromLink());

        // this is what we cannot violate
        for (int s = pickupIdx; s < dropoffIdx; s++) {
            Stop stop = vEntry.stops.get(s);
            stopList.add(stop.getLink());
            // all stops after pickup but still before dropoff are delayed by pickupDetourTimeLoss
            if (stop.task.getBeginTime() + pickupDetourTimeLoss > stop.latestArrivalTime //
                    // (stop.latestArrivalTime is the latest arrival time according to alpha*t_direct + beta
                    // So we are checking if we are now larger than that.)
                    || stop.task.getEndTime() + pickupDetourTimeLoss > stop.latestDepartureTime
                // (this is the same except for the departure. Relates to the maximum waiting. kai, nov'18)
                    || isEllipseConstraintViolated(drtRequest.getFromLink(), stop.getLink(), drtRequest.getToLink())
            ) {
//                LOG.debug("request-ellipse could have been violated");
                return true;
            }
        }
        stopList.add(drtRequest.getToLink());

        // ... now the same for everything after the considered dropoff:

        // this is what we cannot violate
        for (int s = dropoffIdx; s < vEntry.stops.size(); s++) {
            Stop stop = vEntry.stops.get(s);
            // all stops after dropoff are delayed by totalTimeLoss
            if (stop.task.getBeginTime() + totalTimeLoss > stop.latestArrivalTime //
                    || stop.task.getEndTime() + totalTimeLoss > stop.latestDepartureTime) {
                return true;
            }
        }

        // calculating the direct TravelTime
        RouteFactories routeFactories = new RouteFactories();
        routeFactories.setRouteFactory(DrtRoute.class, new DrtRouteFactory());
        DrtRoute directRoute = (DrtRoute) drtRouteCreatorProvider.get().createRoute(drtRequest.getSubmissionTime(),
                drtRequest.getFromLink(),
                drtRequest.getToLink(),
                routeFactories);
        double directTravelTime = directRoute.getDirectRideTime();
        double detourTravelTime = IntStream.range(0, stopList.size() - 1)
                .mapToObj(i -> (DrtRoute) drtRouteCreatorProvider.get().createRoute(drtRequest.getSubmissionTime(),
                        stopList.get(i),
                        stopList.get(i + 1),
                        routeFactories)).mapToDouble(DrtRoute::getDirectRideTime).sum();
        assert detourTravelTime > directTravelTime : "Travel Time of detour route must be higher than travel time of " +
                "direct route!";

        // Comparing direct and indirect traveltime for current request
//        LOG.warn("detourTravelTime: " + detourTravelTime +"\ndirectTravelTime: " + directTravelTime); // TODO:
//         direct times shorter than indirect ones??
//        if (detourTravelTime > RELATIVE_DELIVERY_DELTA*directTravelTime) {
//            LOG.warn("relative_delivery_delay_constraint violated");
//            return true;
//        }

        // vehicle's time window cannot be violated
        DrtStayTask lastTask = (DrtStayTask) Schedules.getLastTask(vEntry.vehicle.getSchedule());
        double timeSlack = vEntry.vehicle.getServiceEndTime() - Math.max(lastTask.getBeginTime(),
                timeOfDay.getAsDouble());
		return timeSlack < totalTimeLoss;// all constraints satisfied
	}

    public static class SoftConstraintViolation {
        public final double maxWaitTimeViolation;
        public final double maxTravelTimeViolation;

        public SoftConstraintViolation(double maxWaitTimeViolation, double maxTravelTimeViolation) {
            this.maxWaitTimeViolation = maxWaitTimeViolation;
            this.maxTravelTimeViolation = maxTravelTimeViolation;
        }
    }

    /**
     * The request constraints are set in {@link DrtRequest}, which is used by {@link DrtRequestCreator},
     * which is used by {@link DrtRouteCreator} and {@link DefaultDrtRouteUpdater}.  kai, nov'18
     */
    private double calcSoftConstraintPenalty(DrtRequest drtRequest, InsertionWithDetourData<D> insertion,
                                             double pickupDetourTimeLoss) {
        VehicleData.Entry vEntry = insertion.getVehicleEntry();
        final int pickupIdx = insertion.getPickup().index;
        final int dropoffIdx = insertion.getDropoff().index;

        double driveToPickupStartTime = getDriveToInsertionStartTime(vEntry, pickupIdx);
        // (normally the end time of the previous task)
        double pickupEndTime = driveToPickupStartTime
                + detourTime.applyAsDouble(insertion.getDetourToPickup())
                + stopDuration;
        double dropoffStartTime = pickupIdx == dropoffIdx ?
                pickupEndTime + detourTime.applyAsDouble(insertion.getDetourFromPickup()) :
                // (special case if inserted dropoff is directly after inserted pickup)
                vEntry.stops.get(dropoffIdx - 1).task.getEndTime() + pickupDetourTimeLoss + detourTime.applyAsDouble(
                        insertion.getDetourToDropoff());

        double maxWaitTimeViolation = Math.max(0, pickupEndTime - drtRequest.getLatestStartTime());
        // how much we are beyond the latest start time = request time + max wait time.
        // max wait time currently comes from config

        double maxTravelTimeViolation = Math.max(0, dropoffStartTime - drtRequest.getLatestArrivalTime());
        // how much we are beyond the latest dropoff time = request time + max travel time.
        // max travel time currently calculated with DrtRouteModule.getMaxTravelTime(unsharedRideTime)

        return penaltyCalculator.calcPenalty(maxWaitTimeViolation, maxTravelTimeViolation);
    }

    private double getDriveToInsertionStartTime(VehicleData.Entry vEntry, int insertionIdx) {
        return vEntry.getWaypoint(insertionIdx).getDepartureTime();
    }

    private boolean isEllipseConstraintViolated(Link previousLink, Link insertedLink, Link nextLink) {
        double directDistance = DistanceUtils.calculateDistance(previousLink.getCoord(), nextLink.getCoord());
        double detourDistance = DistanceUtils.calculateDistance(previousLink.getCoord(), insertedLink.getCoord())
                + DistanceUtils.calculateDistance(insertedLink.getCoord(), nextLink.getCoord());
        boolean result = directDistance > 0 &&
                detourDistance > 0 &&
                detourDistance > DETOUR_DELTA * directDistance;
        assert (detourDistance > 0 && directDistance > 0 && directDistance > detourDistance) :
                "detour distance smaller than direct distance";

        return result;
    }
}

