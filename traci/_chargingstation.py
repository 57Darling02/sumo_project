# -*- coding: utf-8 -*-
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2008-2023 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    _chargingstation.py
# @author  Jakob Erdmann
# @date    2020-06-02

from __future__ import absolute_import
from . import constants as tc
from .domain import Domain


class ChargingStationDomain(Domain):

    def __init__(self):
        Domain.__init__(self, "chargingstation",
                        tc.CMD_GET_CHARGINGSTATION_VARIABLE, tc.CMD_SET_CHARGINGSTATION_VARIABLE,
                        tc.CMD_SUBSCRIBE_CHARGINGSTATION_VARIABLE, tc.RESPONSE_SUBSCRIBE_CHARGINGSTATION_VARIABLE,
                        tc.CMD_SUBSCRIBE_CHARGINGSTATION_CONTEXT, tc.RESPONSE_SUBSCRIBE_CHARGINGSTATION_CONTEXT)

    def getLaneID(self, stopID):
        """getLaneID(string) -> string
        Returns the Lane of this calibrator (if it applies to a single Lane)
        """
        return self._getUniversal(tc.VAR_LANE_ID, stopID)

    def getStartPos(self, stopID):
        """getStartPos(string) -> double

        The starting position of the stop along the Lane measured in m.
        """
        return self._getUniversal(tc.VAR_POSITION, stopID)

    def getEndPos(self, stopID):
        """getEndPos(string) -> double

        The end position of the stop along the Lane measured in m.
        """
        return self._getUniversal(tc.VAR_LANEPOSITION, stopID)

    def getName(self, stopID):
        """getName(string) -> string

        Returns the name of this stop
        """
        return self._getUniversal(tc.VAR_NAME, stopID)

    def getVehicleCount(self, stopID):
        """getVehicleCount(string) -> integer
        Get the total number of vehicles stopped at the named charging station.
        """
        return self._getUniversal(tc.VAR_STOP_STARTING_VEHICLES_NUMBER, stopID)

    def getVehicleIDs(self, stopID):
        """getVehicleIDs(string) -> list(string)
        Get the IDs of vehicles stopped at the named charging station.
        """
        return self._getUniversal(tc.VAR_STOP_STARTING_VEHICLES_IDS, stopID)
