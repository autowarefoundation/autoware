function quat(q0,q1,q2,q3)

  q_length=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);

  q0=q0/q_length;
  q1=q1/q_length;
  q2=q2/q_length;
  q3=q3/q_length;

  phi=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
  theta=asin(2*(q0*q2-q3*q1));
  psii=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

  return phi, theta, psii
end

phi, theta, psi=quat(0.99619,0,0,0.08716)
psi*180/pi


function Quaternion_to_EulerianAngle(x, y, z, w)
  ysqr = y*y

  t0 = +2.0 * (w * x + y*z)
  t1 = +1.0 - 2.0 * (x*x + ysqr)
  X = atan2(t0, t1)*180/pi

  t2 = +2.0 * (w*y - z*x)
  t2 = ((t2 > 1.0) ? 1.0 : t2);
  t2 = ((t2 < -1.0) ? -1.0 : t2);
  Y = asin(t2)*180/pi

  t3 = +2.0 * (w * z + x*y)
  t4 = +1.0 - 2.0 * (ysqr + z*z)
  Z = atan2(t3, t4)*180/pi

  return X, Y, Z
end

type	Quaternion
  x
  y
  z
  w
end

function

function toQuaternion(roll, pitch, yaw)

	t0 = cos(yaw/2)
	t1 = sin(yaw/2)
	t2 = cos(roll/2)
	t3 = sin(roll/2)
	t4 = cos(pitch/2)
	t5 = sin(pitch/2)

  w = t0*t2*t4 + t1*t3*t5
	x = t0*t3*t4 - t1*t2*t5
	y = t0*t2*t5 + t1*t3*t4
	z = t1*t2*t4 - t0*t3*t5

  q = Quaternion(x,y,z,w)

	return q
end

roll = 10
pitch = 0
yaw = 0
toQuaternion(roll, pitch, yaw)
phi, theta, psi=Quaternion_to_EulerianAngle(0.99619,0,0,0.08716)


using PyCall

@pyimport tf.transformations as tf

roll = 0
pitch = 0
yaw = pi/2

Q = tf.quaternion_from_euler(roll, pitch, yaw)

roll, pitch, yaw = tf.euler_from_quaternion(Q)
