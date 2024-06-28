import 'dart:math';

import 'package:robotic_kinematics/kinematics.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';


const discrepancy = 0.0001;

extension EQUAL on double {
  bool equal(double other, {double tolerance = discrepancy}){
    return (this - other).abs() < tolerance;
  }
}

void main() {
  group('Test of Kinematics:', () {
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

    final expectedCord = [
      0.41338, // x
      0.50142, // y
      0.57895, // z
      -3.1078, // rx
      -0.0227, // ry
      0.2642, // rz
    ];



    setUp(() {
      // Additional setup goes here.
    });

    test('Forward', () {

      final endFactor = forwardKinematics(jointAngles, iimt);
      print(endFactor);

      // Extract end-effector position
      final T_final = endFactor;
      Vector3 position = Vector3.zero();
      T_final.transform3(position);

      // Extract end-effector orientation
      final simpleRotation = T_final.getRotation();
      final orientation = Quaternion.fromRotation(simpleRotation);
      print("orientation: $orientation");
      final coord = List<double>.generate(6,(_)=>0.0);
      position.copyIntoArray(coord);
      orientation.eulerAngles.copyIntoArray(coord,3);
      print('Original: $expectedCord\ncalculated: $coord');
      var i=0;
      while(i<expectedCord.length){
        final original=expectedCord[i], actual = coord[i];
        expect(original.equal(actual),true);
        i++;
      }
    });
    test('Inverse', (){
      // Define the target position and orientation (in meters and radians)
      Vector3 targetPosition = Vector3.array(expectedCord);
      Quaternion targetOrientation = Quaternion.axisAngle(Vector3(0, 0, 1), expectedCord[5]) // rz
        *(Quaternion.axisAngle(Vector3(0, 1, 0), expectedCord[4])) // ry
        *(Quaternion.axisAngle(Vector3(1, 0, 0), expectedCord[3]) // rz
          );
      print("position: $targetPosition $targetOrientation");
      print('angles: ${targetOrientation.eulerAngles}');
      // Define the DH parameters
      final dhParameters = iimt;

      // Calculate the joint angles using inverse kinematics
      // List<double> jointAngles = inverseKinematics(targetPosition, targetOrientation, dhParameters);
      // List<double> jointAngles = particleSwarmOptimization(targetPosition, targetOrientation, dhParameters, swarmSize: 30, dimensions: 6, maxIterations: 1000);
      List<double> jointAngles = ik(targetPosition, targetOrientation, dhParameters, swarmSize: 30, dimensions: 6, maxIterations: 1000);
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
