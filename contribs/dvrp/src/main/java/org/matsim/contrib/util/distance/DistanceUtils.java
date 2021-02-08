package org.matsim.contrib.util.distance;

import org.matsim.api.core.v01.BasicLocation;
import org.matsim.api.core.v01.Coord;
import org.matsim.core.utils.geometry.CoordUtils;

public class DistanceUtils {
	public static double calculateDistance(BasicLocation fromLocation, BasicLocation toLocation) {
		return calculateDistance(fromLocation.getCoord(), toLocation.getCoord());
	}

	public static double calculateSquaredDistance(BasicLocation fromLocation, BasicLocation toLocation) {
		return calculateSquaredDistance(fromLocation.getCoord(), toLocation.getCoord());
	}

	/**
	 * @return distance (for distance-based comparison/sorting, consider using the squared distance)
	 */
	public static double calculateDistance(Coord fromCoord, Coord toCoord) {
		return Math.sqrt(calculateSquaredDistance(fromCoord, toCoord));
	}

	/**
	 * @return SQUARED distance (to avoid unnecessary Math.sqrt() calls when comparing distances)
	 */
	public static double calculateSquaredDistance(Coord fromCoord, Coord toCoord) {
		double L = 10000;
		double deltaX = Math.abs(toCoord.getX() - fromCoord.getX());
		deltaX = deltaX < L / 2 ? deltaX : -deltaX + L;
		double deltaY = Math.abs(toCoord.getY() - fromCoord.getY());
		deltaY = deltaY < L / 2 ? deltaY : -deltaY + L;
		return deltaX * deltaX + deltaY * deltaY;
	}
}
