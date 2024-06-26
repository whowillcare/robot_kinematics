import 'package:kinematics/kinematics.dart';

void main() {
  final jointAngles = [-2.32, -1.647, 2.658, -4.088, 4.128, 1.685];

  final dhParameters = [
    DHParameter(0.139, 0, 0),       // d1, a1, alpha1
    DHParameter(0, -0.3945, 0),    // d2, a2, alpha2
    DHParameter(0, -0.373, 0),     // d3, a3, alpha3
    DHParameter(0.1245, 0, 0),     // d4, a4, alpha4
    DHParameter(0.08925, 0, 0),    // d5, a5, alpha5
    DHParameter(0.1031, 0, 0)      // d6, a6, alpha6
  ];

  final T_final = forwardKinematics(jointAngles, dhParameters);

  // Extract end-effector position
  final position = T_final.getTranslation();
  // Extract end-effector orientation
  final orientation = T_final.getRotation();

  print('End-effector position: $position');
  print('End-effector orientation: $orientation');
}