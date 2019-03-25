# EGO

## Clonning with sub-modules

Clone recursively:

`git clone --recursive https://github.com/NMMI/EGO.git`

## Usage, the command to launch EGO robot

The command to launch the sensor:

`roslaunch segway_base_controller sensor.launch`

The command to launch the lqr base control:

`roslaunch segway_base_controller lqr_controller.launch`

The command to launch the upper body:

`roslaunch qb_frank_controller arms_manager_inv_kin.launch`

The command to launch the camera:

`roslaunch zed_cpu_ros zed_cpu_ros.launch`

The command to launch the bridge with the pilot station:

`roslaunch zed_oculus_qb_bridge oculus_joy_bridge.launch`

## Istitutions

`Istituto Italiano di Tecnologia, via Morego, 30, 16163 Genova, Italia`

`Centro di Ricerca E. Piaggio e Dipartimento di Ingegneria dell’Informazione, Universita di Pisa, Pisa, Italia`
