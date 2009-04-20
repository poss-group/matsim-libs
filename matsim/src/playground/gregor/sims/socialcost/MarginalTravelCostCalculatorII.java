/* *********************************************************************** *
 * project: org.matsim.*
 * MarginalTravelCostCalculatorII.java
 *                                                                         *
 * *********************************************************************** *
 *                                                                         *
 * copyright       : (C) 2007 by the members listed in the COPYING,        *
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

package playground.gregor.sims.socialcost;

import org.matsim.core.api.network.Link;
import org.matsim.core.router.util.TravelCost;
import org.matsim.core.trafficmonitoring.AbstractTravelTimeCalculator;

public class MarginalTravelCostCalculatorII implements TravelCost {

	


	private final SocialCostCalculator sc;
	private final AbstractTravelTimeCalculator tc;

	public MarginalTravelCostCalculatorII(final AbstractTravelTimeCalculator tc, final SocialCostCalculator sc) {
		this.tc = tc;
		this.sc = sc;
	}
	

	public double getLinkTravelCost(final Link link, final double time) {
		double t = this.tc.getLinkTravelTime(link, time);
		double s = this.sc.getSocialCost(link, time);
		double cost = t+s;
		if (cost < 0) {
			System.err.println("negative cost:" + cost);
		} else if (Double.isNaN(cost)) {
			System.err.println("nan cost:" + cost);
		} else if (Double.isInfinite(cost)) {
			System.err.println("infinite cost:" + cost);
		} else if (cost > 10000) {
			System.out.println("verry high cost:" + cost);
		}
		return t + s;
	}
	

}