import 'dart:math';

import 'package:vector_math/vector_math_64.dart';
import 'dart:math' as math;

class DHParameter {
  double d;
  double a;
  double alpha;

  DHParameter(this.d, this.a, this.alpha);
}

Vector3  quaternionToEuler(Quaternion q) {
  double ysqr = q.y * q.y;

  // roll (x-axis rotation)
  double t0 = 2.0 * (q.w * q.x + q.y * q.z);
  double t1 = 1.0 - 2.0 * (q.x * q.x + ysqr);
  double roll = math.atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = 2.0 * (q.w * q.y - q.z * q.x);
  t2 = t2.clamp(-1.0, 1.0);
  double pitch = math.asin(t2);

  // yaw (z-axis rotation)
  double t3 = 2.0 * (q.w * q.z + q.x * q.y);
  double t4 = 1.0 - 2.0 * (ysqr + q.z * q.z);
  double yaw = math.atan2(t3, t4);

  return Vector3(roll, pitch, yaw);
}

extension Euler on Quaternion {
    Vector3 get eulerAngles => quaternionToEuler(this);
}

double approx(double number, [double minPositive=1e-16]){
  return  number.abs() < minPositive ? 0 : number;
}
double pSin(double radian) => approx(math.sin(radian));
double pCos(double radian) => approx(math.cos(radian));

Matrix4 forwardKinematics(List<double> jointAngles, List<DHParameter> dhParameters) {
  if (jointAngles.length != dhParameters.length) {
    throw ArgumentError('The number of joint angles must match the number of DH parameters');
  }

  Matrix4? tFinal;

  for (int i = 0; i < jointAngles.length; i++) {
    double theta = jointAngles[i];
    double d = dhParameters[i].d;
    double a = dhParameters[i].a;
    double alpha = dhParameters[i].alpha;

    double cosTheta = pCos(theta);
    double sinTheta = pSin(theta);
    double cosAlpha = pCos(alpha);
    double sinAlpha = pSin(alpha);

    Matrix4 T = Matrix4(
        cosTheta, -sinTheta * cosAlpha,  sinTheta * sinAlpha, a * cosTheta,
        sinTheta,  cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta,
        0,         sinAlpha,             cosAlpha,            d,
        0,         0,                    0,                   1
    ) ..transpose();
    if (tFinal == null){
      tFinal = T;
    }else {
      tFinal = tFinal * T;
    }
  }

  return tFinal ?? Matrix4.zero();
}

// Function to calculate inverse kinematics
List<double> inverseSimple(Vector3 targetPosition, Quaternion targetOrientation, List<DHParameter> dhParameters) {
  // Initialize joint angles list
  List<double> jointAngles = List.filled(6, 0.0);

  // Step 1: Compute the position of the wrist center
  Vector3 d6Vector = Vector3(0, 0, dhParameters.last.d);
  Vector3 wristCenter = targetPosition - targetOrientation.rotate(d6Vector);

  // Step 2: Solve for base and shoulder joints
  jointAngles[0] = math.atan2(wristCenter.y, wristCenter.x);
  double r = math.sqrt(wristCenter.x * wristCenter.x + wristCenter.y * wristCenter.y);
  double s = wristCenter.z - dhParameters[0].d;

  double d2 = dhParameters[1].a;
  double d3 = dhParameters[2].a;

  double cosTheta2 = (r * r + s * s - d2 * d2 - d3 * d3) / (2 * d2 * d3);
  jointAngles[2] = math.acos(cosTheta2);

  double k1 = d2 + d3 * cosTheta2;
  double k2 = d3 * math.sin(jointAngles[2]);
  jointAngles[1] = math.atan2(s, r) - math.atan2(k2, k1);

  // Step 3: Solve for wrist joints based on orientation
  Matrix4 R0_3 = forwardKinematics(jointAngles.sublist(0, 3), dhParameters.sublist(0, 3));
  Matrix4 T_target = Matrix4.compose(targetPosition, targetOrientation, Vector3.all(1));
  Matrix4 R3_6 = R0_3.clone()..invert();
  R3_6 = R3_6 * T_target;

  jointAngles[3] = math.atan2(R3_6.entry(1, 2), R3_6.entry(0, 2));
  jointAngles[4] = math.atan2(math.sqrt(R3_6.entry(0, 2) * R3_6.entry(0, 2) + R3_6.entry(1, 2) * R3_6.entry(1, 2)), R3_6.entry(2, 2));
  jointAngles[5] = math.atan2(R3_6.entry(2, 1), -R3_6.entry(2, 0));

  return jointAngles;
}

List<List<double>> invert6x6Matrix(List<List<double>> matrix) {
  // Implement 6x6 matrix inversion (placeholder with identity matrix for now)
  List<List<double>> identity = List.generate(6, (i) => List.generate(6, (j) => i == j ? 1.0 : 0.0));
  // Replace with actual inversion logic
  return identity;
}

List<double> multiplyMatrixVector(List<List<double>> matrix, List<double> vector) {
  List<double> result = List.filled(6, 0.0);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      result[i] += matrix[i][j] * vector[j];
    }
  }
  return result;
}

List<List<double>> computeJacobian(List<double> jointAngles, List<DHParameter> dhParameters, double perturbation) {
  // Initialize the Jacobian matrix as a 6x6 list
  List<List<double>> J = List.generate(6, (_) => List.filled(6, 0.0));

  // Forward kinematics to get the current end-effector position and orientation
  Matrix4 T_final = forwardKinematics(jointAngles, dhParameters);
  Vector3 currentPosition = T_final.getTranslation();
  Quaternion currentOrientation = Quaternion.fromRotation(T_final.getRotation());

  // Compute the Jacobian columns for each joint
  for (int i = 0; i < jointAngles.length; i++) {
    // Perturb the current joint angle
    List<double> perturbedAngles = List.from(jointAngles);
    perturbedAngles[i] += perturbation;

    // Forward kinematics with the perturbed angle
    Matrix4 T_perturbed = forwardKinematics(perturbedAngles, dhParameters);
    Vector3 perturbedPosition = T_perturbed.getTranslation();

    // Compute the difference in position
    Vector3 deltaPosition = (perturbedPosition - currentPosition) / perturbation;

    // Compute the difference in orientation
    Quaternion perturbedOrientation = Quaternion.fromRotation(T_perturbed.getRotation());
    Quaternion deltaOrientation = perturbedOrientation * currentOrientation.conjugated();

    // Convert deltaOrientation to angle-axis representation
    Vector3 axis = Vector3.zero();
    double angle = 2 * math.acos(deltaOrientation.w);
    if (angle > perturbation) {
      double s = math.sqrt(1 - deltaOrientation.w * deltaOrientation.w);
      if (s < perturbation) {
        axis.setValues(deltaOrientation.x, deltaOrientation.y, deltaOrientation.z);
      } else {
        axis.setValues(deltaOrientation.x / s, deltaOrientation.y / s, deltaOrientation.z / s);
      }
    }

    Vector3 deltaOrientationVector = axis * angle;

    // Construct the Jacobian column for position
    J[0][i] = deltaPosition.x;
    J[1][i] = deltaPosition.y;
    J[2][i] = deltaPosition.z;

    // Construct the Jacobian column for orientation
    J[3][i] = deltaOrientationVector.x;
    J[4][i] = deltaOrientationVector.y;
    J[5][i] = deltaOrientationVector.z;
  }

  return J;
}

List<double> inverseKinematics(Vector3 targetPosition, Quaternion targetOrientation, List<DHParameter> dhParameters, {int maxIterations = 10000, double tolerance = 1e-6, double lambda = 0.1}) {
  // Initialize joint angles list
  List<double> jointAngles = List.filled(6, 0.0);

  for (int iteration = 0; iteration < maxIterations; iteration++) {
    // Forward kinematics to get the current end-effector position and orientation
    Matrix4 T_final = forwardKinematics(jointAngles, dhParameters);
    Vector3 currentPosition = T_final.getTranslation();
    Quaternion currentOrientation = Quaternion.fromRotation(T_final.getRotation());

    // Compute the position and orientation error
    Vector3 positionError = targetPosition - currentPosition;
    Quaternion orientationError = targetOrientation * currentOrientation.conjugated();

    // Convert orientation error to angle-axis representation for easier manipulation
    Vector3 axis = Vector3.zero();
    double angle = 2 * math.acos(orientationError.w);
    if (angle > tolerance) {
      double s = math.sqrt(1 - orientationError.w * orientationError.w);
      if (s < tolerance) {
        axis.setValues(orientationError.x, orientationError.y, orientationError.z);
      } else {
        axis.setValues(orientationError.x / s, orientationError.y / s, orientationError.z / s);
      }
    }

    Vector3 orientationErrorVector = axis * angle;

    // Check if the error is within the tolerance
    if (positionError.length < tolerance && orientationErrorVector.length < tolerance) {
      break;
    }

    print('$iteration -> ${positionError.length} ${orientationErrorVector.length}');
    // Compute the Jacobian matrix
    List<List<double>> J = computeJacobian(jointAngles, dhParameters, 1e-6);

    // Combine position and orientation error vectors into a 6-element list
    List<double> errorVector = [
      positionError.x, positionError.y, positionError.z,
      orientationErrorVector.x, orientationErrorVector.y, orientationErrorVector.z,
    ];

    // Construct the damped Jacobian matrix (J' * J + lambda^2 * I)
    List<List<double>> JT = List.generate(6, (_) => List.filled(6, 0.0));
    List<List<double>> JTJ = List.generate(6, (_) => List.filled(6, 0.0));
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        for (int k = 0; k < 6; k++) {
          JT[i][j] += J[k][i] * J[k][j];
        }
        if (i == j) {
          JTJ[i][j] = JT[i][j] + lambda * lambda;
        } else {
          JTJ[i][j] = JT[i][j];
        }
      }
    }

    // Invert the damped Jacobian matrix
    List<List<double>> JTJ_inv = invert6x6Matrix(JTJ);

    // Compute J_inv * errorVector using the damped inverse Jacobian
    List<double> deltaAngles = multiplyMatrixVector(JTJ_inv, errorVector);

    print('Delta $deltaAngles');
    // Update the joint angles with constraints
    for (int i = 0; i < jointAngles.length; i++) {
      jointAngles[i] += deltaAngles[i];
      jointAngles[i] = jointAngles[i].clamp(-2 * math.pi, 2 * math.pi);
    }
  }

  return jointAngles;
}



class Particle {
  late List<double> position;
  late List<double> velocity;
  late List<double> bestPosition;
  late double bestFitness;

  Particle(int dimensions) {
    position = List<double>.filled(dimensions, 0);
    velocity = List<double>.filled(dimensions, 0);
    bestPosition = List<double>.filled(dimensions, 0);
    bestFitness = double.infinity;
  }
}

List<Particle> initializeSwarm(int swarmSize, int dimensions) {
  List<Particle> swarm = [];
  Random random = Random();

  for (int i = 0; i < swarmSize; i++) {
    Particle particle = Particle(dimensions);
    for (int j = 0; j < dimensions; j++) {
      particle.position[j] = -2 * pi + 4 * pi * random.nextDouble(); // Joint limits
      particle.velocity[j] = -0.1 + 0.2 * random.nextDouble();
    }
    swarm.add(particle);
  }
  return swarm;
}

double objectiveFunction(List<double> jointAngles, Vector3 targetPosition, Quaternion targetOrientation, List<DHParameter> dhParameters) {
  Matrix4 T_final = forwardKinematics(jointAngles, dhParameters);
  Vector3 currentPosition = T_final.getTranslation();
  Quaternion currentOrientation = Quaternion.fromRotation(T_final.getRotation());

  Vector3 positionError = targetPosition - currentPosition;
  Quaternion orientationError = targetOrientation * currentOrientation.conjugated();

  Vector3 axis = Vector3.zero();
  double angle = 2 * acos(orientationError.w);
  if (angle > 1e-6) {
    double s = sqrt(1 - orientationError.w * orientationError.w);
    if (s < 1e-6) {
      axis.setValues(orientationError.x, orientationError.y, orientationError.z);
    } else {
      axis.setValues(orientationError.x / s, orientationError.y / s, orientationError.z / s);
    }
  }
  Vector3 orientationErrorVector = axis * angle;

  double positionErrorNorm = positionError.length;
  double orientationErrorNorm = orientationErrorVector.length;

  return positionErrorNorm + orientationErrorNorm;
}


List<double> particleSwarmOptimization(Vector3 targetPosition, Quaternion targetOrientation, List<DHParameter> dhParameters, {int swarmSize = 30, int dimensions = 6, int maxIterations = 1000, double tolerance = 1e-6}) {
  List<Particle> swarm = initializeSwarm(swarmSize, dimensions);
  List<double> globalBestPosition = List<double>.filled(dimensions, 0);
  double globalBestFitness = double.infinity;
  double w = 0.9; // Start with a higher inertia weight
  double wMin = 0.4; // Minimum inertia weight
  double c1 = 2.0; // Cognitive coefficient
  double c2 = 2.0; // Social coefficient
  Random random = Random();

  for (int iteration = 0; iteration < maxIterations; iteration++) {
    for (Particle particle in swarm) {
      double fitness = objectiveFunction(particle.position, targetPosition, targetOrientation, dhParameters);
      if (fitness < particle.bestFitness) {
        particle.bestFitness = fitness;
        particle.bestPosition = List.from(particle.position);
      }
      if (fitness < globalBestFitness) {
        globalBestFitness = fitness;
        globalBestPosition = List.from(particle.position);
      }
      if (globalBestFitness < tolerance) {
        print('Converged at iteration $iteration with fitness $globalBestFitness');
        break;
      }
    }

    // Check if the solution is within the tolerance
    if (globalBestFitness < tolerance) {
      print('Converged at iteration $iteration with fitness $globalBestFitness');
      break;
    }else {
      //print('Continue iteration $iteration with fitness $globalBestFitness');
    }

    // Linearly decreasing inertia weight
    w = wMin + (0.9 - wMin) * (maxIterations - iteration) / maxIterations;

    for (Particle particle in swarm) {
      for (int j = 0; j < dimensions; j++) {
        double r1 = random.nextDouble();
        double r2 = random.nextDouble();
        particle.velocity[j] = w * particle.velocity[j] + c1 * r1 * (particle.bestPosition[j] - particle.position[j]) + c2 * r2 * (globalBestPosition[j] - particle.position[j]);
        particle.position[j] += particle.velocity[j];
        particle.position[j] = particle.position[j].clamp(-2 * pi, 2 * pi); // Joint limits
      }
    }
  }
  return globalBestPosition;
}


List<double> optimizeFirstThreeJoints(Vector3 targetPosition, List<DHParameter> dhParameters) {
  // Objective function for the first three joints
  double positionObjectiveFunction(List<double> jointAngles) {
    // Use the first three joints for forward kinematics to get the current position
    List<double> angles = jointAngles + [0.0, 0.0, 0.0]; // Extend to 6 DOF with zeros for the last three
    Matrix4 T_final = forwardKinematics(angles, dhParameters);
    Vector3 currentPosition = T_final.getTranslation();
    return (targetPosition - currentPosition).length;
  }

  // Initial guess for the first three joints
  List<double> initialJointAngles = [0.0, 0.0, 0.0];

  // Optimization using PSO (or another optimizer)
  List<double> optimizedJointAngles = particleSwarmOptimizationForSubset(positionObjectiveFunction, initialJointAngles, 3);

  return optimizedJointAngles;
}

List<double> optimizeLastThreeJoints(Quaternion targetOrientation, List<DHParameter> dhParameters, List<double> firstThreeJointAngles) {
  // Objective function for the last three joints
  double orientationObjectiveFunction(List<double> jointAngles) {
    // Combine the first three joint angles with the current last three
    List<double> angles = firstThreeJointAngles + jointAngles;
    Matrix4 T_final = forwardKinematics(angles, dhParameters);
    Quaternion currentOrientation = Quaternion.fromRotation(T_final.getRotation());
    Quaternion orientationError = targetOrientation * currentOrientation.conjugated();

    Vector3 axis = Vector3.zero();
    double angle = 2 * acos(orientationError.w);
    if (angle > 1e-6) {
      double s = sqrt(1 - orientationError.w * orientationError.w);
      if (s < 1e-6) {
        axis.setValues(orientationError.x, orientationError.y, orientationError.z);
      } else {
        axis.setValues(orientationError.x / s, orientationError.y / s, orientationError.z / s);
      }
    }
    return axis.length * angle;
  }

  // Initial guess for the last three joints
  List<double> initialJointAngles = [0.0, 0.0, 0.0];

  // Optimization using PSO (or another optimizer)
  List<double> optimizedJointAngles = particleSwarmOptimizationForSubset(orientationObjectiveFunction, initialJointAngles, 3);

  return optimizedJointAngles;
}


List<double> particleSwarmOptimizationForSubset(Function objectiveFunction, List<double> initialJointAngles, int dimensions, {int swarmSize = 30, int maxIterations = 1000, double tolerance = 1e-6}) {
  List<Particle> swarm = initializeSwarm(swarmSize, dimensions);
  List<double> globalBestPosition = List<double>.filled(dimensions, 0);
  double globalBestFitness = double.infinity;
  double w = 0.9; // Start with a higher inertia weight
  double wMin = 0.4; // Minimum inertia weight
  double c1 = 2.0; // Cognitive coefficient
  double c2 = 2.0; // Social coefficient
  Random random = Random();

  for (int iteration = 0; iteration < maxIterations; iteration++) {
    for (Particle particle in swarm) {
      double fitness = objectiveFunction(particle.position);
      if (fitness < particle.bestFitness) {
        particle.bestFitness = fitness;
        particle.bestPosition = List.from(particle.position);
      }
      if (fitness < globalBestFitness) {
        globalBestFitness = fitness;
        globalBestPosition = List.from(particle.position);
      }
    }

    // Check if the solution is within the tolerance
    if (globalBestFitness < tolerance) {
      print('Converged at iteration $iteration with fitness $globalBestFitness');
      break;
    }

    // Linearly decreasing inertia weight
    w = wMin + (0.9 - wMin) * (maxIterations - iteration) / maxIterations;

    for (Particle particle in swarm) {
      for (int j = 0; j < dimensions; j++) {
        double r1 = random.nextDouble();
        double r2 = random.nextDouble();
        particle.velocity[j] = w * particle.velocity[j] + c1 * r1 * (particle.bestPosition[j] - particle.position[j]) + c2 * r2 * (globalBestPosition[j] - particle.position[j]);
        particle.position[j] += particle.velocity[j];
        particle.position[j] = particle.position[j].clamp(-2 * pi, 2 * pi); // Joint limits
      }
    }
  }
  return globalBestPosition;
}

List<double> ik(Vector3 targetPosition, Quaternion targetOrientation, List<DHParameter> dhParameters, {int swarmSize = 30, int dimensions = 6, int maxIterations = 1000, double tolerance = 1e-6}) {

  // Objective function for the first three joints
  double positionObjectiveFunction(List<double> jointAngles) {
    // Use the first three joints for forward kinematics to get the current position
    List<double> angles = jointAngles + [0.0, 0.0, 0.0]; // Extend to 6 DOF with zeros for the last three
    Matrix4 T_final = forwardKinematics(angles, dhParameters);
    Vector3 currentPosition = T_final.getTranslation();
    return (targetPosition - currentPosition).length;
  }
  Function orientationObjectiveFunctionPrepare(List<double> firstThreeJointAngles) {
  return
    (List<double> jointAngles) {
      // Combine the first three joint angles with the current last three
      List<double> angles = firstThreeJointAngles + jointAngles;
      Matrix4 T_final = forwardKinematics(angles, dhParameters);
      Quaternion currentOrientation = Quaternion.fromRotation(T_final.getRotation());
      Quaternion orientationError = targetOrientation * currentOrientation.conjugated();

      Vector3 axis = Vector3.zero();
      double angle = 2 * acos(orientationError.w);
      if (angle > 1e-6) {
        double s = sqrt(1 - orientationError.w * orientationError.w);
        if (s < 1e-6) {
          axis.setValues(orientationError.x, orientationError.y, orientationError.z);
        } else {
          axis.setValues(orientationError.x / s, orientationError.y / s, orientationError.z / s);
        }
      }
      return axis.length * angle;
    };
  }

  late List<double> firstThreeJointAngles, lastThreeJointAngles;
  {
    // Initial guess for the first three joints
    List<double> initialJointAngles = [0.0, 0.0, 0.0];

    // Optimization using PSO (or another optimizer)
    List<double> optimizedJointAngles = particleSwarmOptimizationForSubset(
        positionObjectiveFunction, initialJointAngles, 3, swarmSize: swarmSize, maxIterations: maxIterations, tolerance: tolerance);
    firstThreeJointAngles = optimizedJointAngles;
  }

  {
    // Initial guess for the last three joints
    List<double> initialJointAngles = [0.0, 0.0, 0.0];
    // Optimization using PSO (or another optimizer)
    List<double> optimizedJointAngles = particleSwarmOptimizationForSubset(
        orientationObjectiveFunctionPrepare(firstThreeJointAngles), initialJointAngles, 3, swarmSize: swarmSize, maxIterations: maxIterations, tolerance: tolerance);
    lastThreeJointAngles = optimizedJointAngles;
  }

  return firstThreeJointAngles + lastThreeJointAngles;
}
