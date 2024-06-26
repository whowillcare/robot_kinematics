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

Matrix4 computeJacobian(List<double> jointAngles, List<DHParameter> dhParameters) {
  // Initialize the Jacobian matrix (6x6)
  List<Vector4> columns = List.generate(6, (_) => Vector4.zero());

  // Forward kinematics to get the current end-effector position and orientation
  Matrix4 T_final = forwardKinematics(jointAngles, dhParameters);
  Vector3 currentPosition = T_final.getTranslation();
  Quaternion currentOrientation = Quaternion.fromRotation(T_final.getRotation());

  // Compute the Jacobian columns for each joint
  for (int i = 0; i < jointAngles.length; i++) {
    // Perturb the current joint angle
    List<double> perturbedAngles = List.from(jointAngles);
    perturbedAngles[i] += 0.001;

    // Forward kinematics with the perturbed angle
    Matrix4 T_perturbed = forwardKinematics(perturbedAngles, dhParameters);
    Vector3 perturbedPosition = T_perturbed.getTranslation();

    // Compute the difference in position
    Vector3 deltaPosition = (perturbedPosition - currentPosition) / 0.001;

    // Compute the difference in orientation
    Quaternion perturbedOrientation = Quaternion.fromRotation(T_perturbed.getRotation());
    Quaternion deltaOrientation = perturbedOrientation - currentOrientation;

    // Construct the Jacobian column for position
    columns[i] = Vector4(deltaPosition.x, deltaPosition.y, deltaPosition.z, 0.0);

    // Construct the Jacobian column for orientation
    if (i < 3) {
      columns[i + 3] = Vector4(deltaOrientation.x, deltaOrientation.y, deltaOrientation.z, 0.0);
    }
  }

  return Matrix4.columns(columns[0], columns[1], columns[2], columns[3]);
}

List<double> inverseKinematics(Vector3 targetPosition, Quaternion targetOrientation, List<DHParameter> dhParameters, {int maxIterations = 100, double tolerance = 1e-6}) {
  // Initialize joint angles list
  List<double> jointAngles = List.filled(6, 0.0);

  for (int iteration = 0; iteration < maxIterations; iteration++) {
    // Forward kinematics to get the current end-effector position and orientation
    Matrix4 T_final = forwardKinematics(jointAngles, dhParameters);
    Vector3 currentPosition = T_final.getTranslation();
    Quaternion currentOrientation = Quaternion.fromRotation(T_final.getRotation());

    // Compute the position and orientation error
    Vector3 positionError = targetPosition - currentPosition;
    Quaternion orientationError = targetOrientation * (currentOrientation..conjugate());

    // Check if the error is within the tolerance
    if (positionError.length < tolerance && orientationError.length < tolerance) {
      break;
    }

    // Compute the Jacobian matrix
    Matrix4 J = computeJacobian(jointAngles, dhParameters);

    // Compute the change in joint angles
    Vector4 errorVector = Vector4(positionError.x, positionError.y, positionError.z, 0.0);
    Matrix4 J_inv = J.clone()..invert();
    Vector4 deltaAngles = J_inv.transform(errorVector);

    // Update the joint angles
    for (int i = 0; i < jointAngles.length; i++) {
      if (i < 3) {
        jointAngles[i] += deltaAngles[i];
      } else {
        jointAngles[i] += deltaAngles[i - 3];
      }
    }
  }

  return jointAngles;
}