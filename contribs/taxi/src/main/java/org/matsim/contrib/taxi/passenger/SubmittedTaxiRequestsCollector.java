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

package org.matsim.contrib.taxi.passenger;

import java.util.Collections;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.matsim.api.core.v01.Id;
import org.matsim.contrib.dvrp.optimizer.Request;

//TODO remove this class once taxi stats are refactored
public class SubmittedTaxiRequestsCollector {
	private final Map<Id<Request>, TaxiRequest> requests = new ConcurrentHashMap<>();

	public Map<Id<Request>, ? extends TaxiRequest> getRequests() {
		return Collections.unmodifiableMap(requests);
	}

	void addRequest(TaxiRequest request) {
		requests.put(request.getId(), request);
	}
}
