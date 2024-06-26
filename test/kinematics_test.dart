import 'dart:math';

import 'package:kinematics/kinematics.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  group('A group of tests', () {
    final halfPi = pi/2.0;
    final iimt = [
      DHParameter(0.139, 0, halfPi),       // d1, a1, alpha1
      DHParameter(0, -0.3945, 0),    // d2, a2, alpha2
      DHParameter(0, -0.373, 0),     // d3, a3, alpha3
      DHParameter(0.1245, 0, halfPi),     // d4, a4, alpha4
      DHParameter(0.08925, 0, -halfPi),    // d5, a5, alpha5
      DHParameter(0.1031, 0, 0)      // d6, a6, alpha6
    ];

    final jointAngles = [
      -2.4565, // base
      -0.8205, // shoulder
      0.0812, // elbow
      -0.866, // wrist1
      -1.5492, // wrist2
      -1.1499, // wrist3
    ];

    setUp(() {
      // Additional setup goes here.
    });

    test('First Test', () {
      final endFactor = forwardKinematics(jointAngles, iimt);
      print(endFactor);

      // Extract end-effector position
      final T_final = endFactor;
      Vector3 position = Vector3.zero();
      T_final.transform3(position);

      // Extract end-effector orientation
      final simpleRotation = T_final.getRotation();
      final orientation = Quaternion.fromRotation(simpleRotation);

      print('End-effector position: $position');
      print('End-effector simple: $simpleRotation');
      print('End-effector orientation: $orientation');
      print('Angles: ${orientation.eulerAngles}');
    });
    test('inverse', (){
      // Define the target position and orientation (in meters and radians)
      Vector3 targetPosition = Vector3(0.41338, 0.50142, 0.57895);
      Quaternion targetOrientation = Quaternion.axisAngle(Vector3(0, 0, 1), 0.2642)
        *(Quaternion.axisAngle(Vector3(0, 1, 0), -0.0227))
        *(Quaternion.axisAngle(Vector3(1, 0, 0), -3.1078));
      print("position: $targetPosition $targetOrientation");
      // Define the DH parameters
      final dhParameters = iimt;

      // Calculate the joint angles using inverse kinematics
      List<double> jointAngles = inverseKinematics(targetPosition, targetOrientation, dhParameters);

      // Print the joint angles
      print('Joint Angles: $jointAngles');

      // Verify the result by calculating the forward kinematics
      Matrix4 T_final = forwardKinematics(jointAngles, dhParameters);

      // Extract end-effector position
      final position = Vector3.zero();
      T_final.transform3(position);

      // Extract end-effector orientation
      final orientation = Quaternion.fromRotation(T_final.getRotation());

      print('End-effector position: $position');
      print('End-effector orientation: $orientation');
    });
  });
}
