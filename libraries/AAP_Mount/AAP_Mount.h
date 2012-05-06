#ifndef AAP_Mount_h
#define AAP_Mount_h

#include "../AP_Math/AP_Math.h"
#include "../AP_Math/matrix3.h"
#include "../AP_Math/polygon.h"
#include "../AP_Math/vector2.h"
#include "../AP_Math/vector3.h"

#include "../APM_RC/APM_RC.h" // ArduPilot Mega RC Library
#include "../AAP_IRCamera/AAP_IRCamera.h"

#define PITCH_DEGS_PER_PIXEL 0.044f
#define YAW_DEGS_PER_PIXEL 0.043f
#define MAX_MOVE 30

class AAP_Mount
{
	private:
		float pan, tilt;
		int panServoCh, tiltServoCh;
		APM_RC_APM1* APM_RC;
		AAP_IRCamera* IRCamera;

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
		AAP_Mount() {}

		void init(APM_RC_APM1* p_APM_RC, AAP_IRCamera* p_IRCamera, int p_panServoCh, int p_tiltServoCh) {
			APM_RC = p_APM_RC;
			IRCamera = p_IRCamera;
			panServoCh = p_panServoCh;
			tiltServoCh = p_tiltServoCh;
		}

		void forwardKinematics(const Matrix3f& reference_frame, Vector3f& track, const float& pan, const float& tilt) {
			Matrix3f calc_rotation = Matrix3f(ToRad(pan), reference_frame.col(2)) * reference_frame;
			calc_rotation = Matrix3f(ToRad(90.0-tilt), calc_rotation.col(0)) * calc_rotation;
			track = calc_rotation.col(1);
		}

		void forwardKinematics(const Matrix3f& reference_frame, Matrix3f& rotation, const float& pan, const float& tilt) {
			rotation = Matrix3f(ToRad(pan), reference_frame.col(2)) * reference_frame;
			rotation = Matrix3f(ToRad(tilt), rotation.col(0)) * rotation;
			rotation = Matrix3f(ToRad(180.0), rotation.col(1)) * rotation;
			rotation = Matrix3f(ToRad(180.0), rotation.col(0)) * rotation;
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

		// clamp x into the interval [a, b]
		inline void clamp(float& x, const float& a, const float& b)
		{
			if (x < a) x = a;
			else if (x > b) x = b;
		}

		void update(Vector2i track_pixel) {
			float pitch = getTilt();
			float yaw = getPan();
			int xError = track_pixel.x - 512;
			int yError = track_pixel.y - 384;
			// servo velocity
			float dPitch = 0.12*PITCH_DEGS_PER_PIXEL*float(-yError);
			float dYaw = 0.12*YAW_DEGS_PER_PIXEL*float(-xError);
			clamp(dPitch, -MAX_MOVE, MAX_MOVE);
			clamp(dYaw, -MAX_MOVE, MAX_MOVE);
			pitch += dPitch;
			yaw += dYaw;
			setTilt(pitch);
			setPan(yaw);
		}

		void update(float yaw, float pitch, float roll, Vector3f& camera_position) {
			//APM's rotation
		    Matrix3f APM_rotation = Matrix3f(-ToRad(yaw), Vector3f(0.0, 0.0, 1.0))
		                        	* Matrix3f(ToRad(pitch), Vector3f(1.0, 0.0, 0.0))
		                        	* Matrix3f(ToRad(roll), Vector3f(0.0, 1.0, 0.0));

		    //Adjust the pitch shift from the APM to the mount's base
		    Matrix3f mount_rotation = Matrix3f(ToRad(-65.0), APM_rotation.col(0)) * APM_rotation;

		    Matrix3f camera_rotation;
		    forwardKinematics(mount_rotation, camera_rotation, getPan(), getTilt());
		    IRCamera->getTransform2(camera_position, camera_rotation);

		    //Get the IR positions in order to update the mount's orientation
		    //Get the raw IR positions
		    Vector2i ir_pos_raw[4];
		    IRCamera->getRawData(ir_pos_raw);
		    //Filter out the valid IR positions
		    vector<Vector2f> ir_pos;
		    for (int i=0; i<4; i++)
		      if ((ir_pos_raw[i].x != 1023) && (ir_pos_raw[i].y != 1023))
			ir_pos.push_back(Vector2f((float) ir_pos_raw[i].x, (float) ir_pos_raw[i].y));

		    //Update the mount's orientation
		    if (ir_pos.size() > 0) {
				//Average of the IR sources positions
				Vector2f mean_ir_pos(0.0, 0.0);
				for (int i=0; i<ir_pos.size(); i++)
				mean_ir_pos += ir_pos[i];
				mean_ir_pos /= ((float) ir_pos.size());
				//Track this position
				update(Vector2i((int) mean_ir_pos.x, (int) mean_ir_pos.y));
		    } else {
				setPan(0.0);
				setTilt(60.0);
		    }
		}
};
#endif
