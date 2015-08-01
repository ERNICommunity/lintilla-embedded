/*
 * Traction.h
 *
 *  Created on: 27.07.2014
 *      Author: niklausd
 */

#ifndef TRACTION_H_
#define TRACTION_H_

class DbgTrace_Port;
class MotorPWM;
class Timer;

//-----------------------------------------------------------------------------

class TractionAdapter
{
public:
  virtual float getYawAngle() = 0;
  virtual void setTargetAngle(double targetAngle) = 0;
  virtual float computeSpeedDiff() = 0;
  virtual void notifyDirectionChange(bool isForward) = 0;
  virtual ~TractionAdapter() { }

protected:
  TractionAdapter() { }

private: // forbidden default functions
  TractionAdapter& operator= (const TractionAdapter& src);  // assignment operator
  TractionAdapter(const TractionAdapter& src);              // copy constructor
};

//-----------------------------------------------------------------------------

class Traction
{
public:
  Traction(TractionAdapter* adapter = 0);
  virtual ~Traction();

  void attachAdapter(TractionAdapter* adapter);
  TractionAdapter* adapter();

public:
  void motorStop();
  void moveBackward();
  void moveForward();
  void moveControlledForward();
  void readjustAngle();
  void moveStraight(bool forward);
  void spinOnPlace(bool right, float angle);

private:
  void updateActors();

private:
  MotorPWM* m_motorL;
  MotorPWM* m_motorR;
  TractionAdapter* m_adapter;
  DbgTrace_Port* m_tracePort;

  // value for motor speed
  int m_speed_value_motor_left;
  int m_speed_value_motor_right;
  bool m_isLeftMotorFwd;
  bool m_isRightMotorFwd;
  float m_targetAngle;
  Timer* m_pidTimer;

  // Constants
  static const int MOTOR_SPEED_STRAIGHT;
  static const int MOTOR_SPIN_SPEED;

  // H-bridge enable pin for speed control
  static const int SPEED_PIN1;
  static const int SPEED_PIN2;

  // H-bridge leg 1
  static const int MOTOR_1A_PIN;
  static const int MOTOR_3A_PIN;

  // H-bridge leg 2
  static const int MOTOR_2A_PIN;
  static const int MOTOR_4A_PIN;

  static const unsigned int PID_SAMPLING_RATE;

private: // forbidden default functions
  Traction& operator= (const Traction& src);  // assignment operator
  Traction(const Traction& src);              // copy constructor
};

#endif /* TRACTION_H_ */
