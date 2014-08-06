/*
 * Traction.h
 *
 *  Created on: 27.07.2014
 *      Author: niklausd
 */

#ifndef TRACTION_H_
#define TRACTION_H_

class MotorPWM;
class Timer;

//-----------------------------------------------------------------------------

class TractionAdapter
{
public:
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
  void moveStraight(bool forward);
  void spinOnPlace(bool right, float angle);

private:
  void updateActors();

private:
  MotorPWM* m_motorL;
  MotorPWM* m_motorR;
  TractionAdapter* m_adapter;

  // value for motor speed
  int m_speed_value_motor_left;
  int m_speed_value_motor_right;
  bool m_isLeftMotorFwd;
  bool m_isRightMotorFwd ;

  // Constants
  static const int cSpeed;
  static const int cSpinSpeed;

  // H-bridge enable pin for speed control
  static const int speedPin1;
  static const int speedPin2;

  // H-bridge leg 1
  static const int motor1APin;
  static const int motor3APin;

  // H-bridge leg 2
  static int motor2APin;
  static int motor4APin;

private: // forbidden default functions
  Traction& operator= (const Traction& src);  // assignment operator
  Traction(const Traction& src);              // copy constructor
};

#endif /* TRACTION_H_ */
