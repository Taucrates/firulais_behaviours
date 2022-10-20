/*
* This file is part of firulais_behaviours.
*
* Copyright (C) 2022 Antoni Tauler-Rosselló <a.tauler@uib.cat> (University of the Balearic Islands)
*
* firulais_behaviours is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* firulais_behaviours is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with firulais_behaviours. If not, see <http://www.gnu.org/licenses/>.
*/

#include "firulais_behaviours/safety_control.h"

int main(int argc, char** argv) {
  ros::init (argc, argv, "safety_control");
  ros::NodeHandle nh("~");

  firulais_behaviours::SafetyControl sc(nh);

  ros::spin();
  return 0;
}