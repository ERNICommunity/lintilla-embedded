/*
 * LintillaMmiScreen.h
 *
 *  Created on: 15.09.2014
 *      Author: niklausd
 */

#ifndef LINTILLAMMISCREEN_H_
#define LINTILLAMMISCREEN_H_

class LintillaMmi;

class LintillaMmiScreen
{
protected:
  LintillaMmiScreen(LintillaMmi* mmi);
public:
  virtual ~LintillaMmiScreen();

  virtual void updateDisplay()  = 0;
  virtual void setEditMode(bool isEditMode) { m_isEditMode = isEditMode; }
  virtual bool isEditMode() { return m_isEditMode; }
  virtual void setCursorUp()    { }
  virtual void setCursorDown()  { }
  virtual void setCursorRight() { }
  virtual void setCursorLeft()  { }

  LintillaMmi* mmi() { return m_mmi; }

private:
  bool m_isEditMode;
  LintillaMmi* m_mmi;

private: // forbidden default functions
  LintillaMmiScreen& operator = (const LintillaMmiScreen& src);  // assignment operator
  LintillaMmiScreen(const LintillaMmiScreen& src);               // copy constructor
};

//-----------------------------------------------------------------------------

class LintillaMmiHomeScreen : public LintillaMmiScreen
{
public:
  LintillaMmiHomeScreen(LintillaMmi* mmi);
  virtual ~LintillaMmiHomeScreen();
  virtual void updateDisplay();
  virtual void setCursorUp();
  virtual void setCursorDown();

private: // forbidden default functions
  LintillaMmiHomeScreen& operator = (const LintillaMmiHomeScreen& src);  // assignment operator
  LintillaMmiHomeScreen(const LintillaMmiHomeScreen& src);               // copy constructor
};

//-----------------------------------------------------------------------------

class LintillaMmiIdScreen : public LintillaMmiScreen
{
public:
  LintillaMmiIdScreen(LintillaMmi* mmi);
  virtual ~LintillaMmiIdScreen();
  virtual void updateDisplay();
  virtual void setCursorUp();
  virtual void setCursorDown();

private: // forbidden default functions
  LintillaMmiIdScreen& operator = (const LintillaMmiIdScreen& src);  // assignment operator
  LintillaMmiIdScreen(const LintillaMmiIdScreen& src);               // copy constructor
};

//-----------------------------------------------------------------------------

class LintillaMmiWLANScreen : public LintillaMmiScreen
{
public:
  LintillaMmiWLANScreen(LintillaMmi* mmi);
  virtual ~LintillaMmiWLANScreen();
  virtual void updateDisplay();
  virtual void setCursorUp();
  virtual void setCursorDown();

private:
  char* m_ssid;
  char* m_pass;

private: // forbidden default functions
  LintillaMmiWLANScreen& operator = (const LintillaMmiWLANScreen& src);  // assignment operator
  LintillaMmiWLANScreen(const LintillaMmiWLANScreen& src);               // copy constructor
};

#endif /* LINTILLAMMISCREEN_H_ */
