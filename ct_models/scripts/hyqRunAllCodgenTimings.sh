#!/bin/bash

printf "=============== FORWARD DYNAMICS ===============\n"
rosrun ct_models HyQcompareForwardReverseFD
printf "\n \n"

printf "=============== INVERSE DYNAMICS ===============\n"
rosrun ct_models HyQcompareForwardReverseID
printf "\n \n"

printf "=============== FORWARD KINEMATICS ===============\n"
rosrun ct_models HyQcompareForwardReverseKin
printf "\n \n"
