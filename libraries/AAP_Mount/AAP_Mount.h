#ifndef AAP_Mount_h
#define AAP_Mount_h

#include "../AP_Math/AP_Math.h"
#include "../AP_Math/matrix3.h"
#include "../AP_Math/polygon.h"
#include "../AP_Math/vector2.h"
#include "../AP_Math/vector3.h"

#include "../APM_RC/APM_RC.h" // ArduPilot Mega RC Library

class AAP_Mount
{
	private:
		float pan, tilt;
		int panServoCh, tiltServoCh;
		APM_RC_APM1* APM_RC;

		/** Returns the mapped integer of "value". This linear map F is defined such that:
		  * F(value_min) = mapped_min
		  * F(value_max) = mapped_max
		  **/
		int linearMap(int value, int value_min, int value_max, int mapped_min, int mapped_max) {
			return mapped_min + (((float)(value - value_min))/((float)(value_max-value_min)))*(mapped_max-mapped_min);
		}

		int panAngleToPW(int angle) {
			return linearMap(angle, -45+13, 45+13, 1000, 2000);
		}

		int tiltAngleToPW(int angle) {
			angle = 90-angle;
			int servo_angle = 0.005404 * pow(angle, 2.0) + 1.54987 * ((float) angle) - 3.02604;
			return linearMap(servo_angle, -45-1, 45-1, 2000, 1000);
		}

		/*
		int servoToTiltAngle(int servoAngle) {
		  return -0.001468 * pow(servoAngle, 2.0) + 0.64516 * ((float) servoAngle) + 1.95896;
		}

		int tiltToServoAngle(int tiltAngle) {
		  return 0.005404 * pow(tiltAngle, 2.0) + 1.54987 * ((float) tiltAngle) - 3.02604;
		}
		*/

	public:
		AAP_Mount() {
			pan = 0.0;
			tilt = 90.0;
			APM_RC = NULL;
		}

		void init(APM_RC_APM1* p_APM_RC, int p_panServoCh, int p_tiltServoCh) {
			APM_RC = p_APM_RC;
			panServoCh = p_panServoCh;
			tiltServoCh = p_tiltServoCh;
		}

		void forwardKinematics(const Matrix3f& reference_frame, Vector3f& track, const float& pan, const float& tilt) {
			Matrix3f calc_rotation = Matrix3f(ToRad(pan), reference_frame.col(2)) * reference_frame;
			calc_rotation = Matrix3f(ToRad(tilt), calc_rotation.col(0)) * calc_rotation;
			track = calc_rotation.col(1);
		}

		void forwardKinematics(const Matrix3f& reference_frame, Matrix3f& rotation, const float& pan, const float& tilt) {
			rotation = Matrix3f(ToRad(pan), reference_frame.col(2)) * reference_frame;
			rotation = Matrix3f(ToRad(tilt), rotation.col(0)) * rotation;
		}

		void inverseKinematics(const Matrix3f& reference_frame, const Vector3f& track, float& pan, float& tilt) {
			Vector3f local_track = reference_frame.transposed() * track;
			pan = ToDeg(atan2(-local_track.x, local_track.y));
			tilt = ToDeg(atan2(-local_track.z, sqrt(pow(local_track.x,2.0)+pow(local_track.y,2.0))));
			if (pan > 90)
			pan -= 180;
			else if (pan < -90)
			pan += 180;
			else
			tilt *= -1;
			if (tilt < 0)
			tilt += 180;
			if (local_track.z < 0)
			tilt += 180;
		}

		float setPan(float p_pan) {
			// limits the change in pan
			if (abs(p_pan-pan) > 10.0) {
				if (p_pan > pan)	pan += 10.0;
				else				pan -= 10.0;
			} else {
				pan = p_pan;
			}
			if (pan > 58.9) pan = 58.9;
			else if (pan < -32.9) pan = -32.9;
			APM_RC->OutputCh(panServoCh, panAngleToPW(int(pan)));
			return pan;
		}

		float setTilt(float p_tilt) {
			// limits the change in tilt
			if (abs(p_tilt-tilt) > 10.0) {
				if (p_tilt > tilt)	tilt += 10.0;
				else				tilt -= 10.0;
			} else {
				tilt = p_tilt;
			}
			if (tilt > 121.9) tilt = 121.9;
			else if (tilt < 62.0) tilt = 62.0;
			APM_RC->OutputCh(tiltServoCh, tiltAngleToPW(int(tilt)));
			return tilt;
		}

		float getPan() { return pan; }
		float getTilt() { return tilt; }

		void update(const Matrix3f& mount_rotation, const Vector3f& track) {
			float new_pan, new_tilt;
			inverseKinematics(mount_rotation, track, new_pan, new_tilt);
			Vector3f calc_track, new_calc_track;
			forwardKinematics(mount_rotation, calc_track, pan, tilt);
			forwardKinematics(mount_rotation, new_calc_track, new_pan, new_tilt);

			if (new_pan > 58.9) new_pan = 58.9;
			else if (new_pan < -32.9) new_pan = -32.9;
			if (new_tilt > 121.9) new_tilt = 121.9;
			else if (new_tilt < 62.0) new_tilt = 62.0;

			float angle_diff = ToDeg(acos((calc_track.normalized()) * (new_calc_track.normalized())));
			if (angle_diff > 5.0) {
				setPan(new_pan);
				setTilt(new_tilt);
			}
		}
};
#endif
