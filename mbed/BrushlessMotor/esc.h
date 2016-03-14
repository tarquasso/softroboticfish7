#ifndef _ESC_H_
#define _ESC_H_

// ESC class used to controll standard Electronic Speed Controllers for brushless motors of RC models.
//  Simple usage example:
//  @code

 
class ESC
{    
  private:
  
    PwmOut esc;
    int period;
    int throttle;
    
  public:
  
    /** Initializes the PwmOut for minimum throttle (1000us).
     *  @param pwmPinOut is the pin connected to the ESC.
     *  @param period is the PWM period in ms (default is 20ms).
     */
    ESC (const PinName pwmPinOut, const int period=20);
    
    /** Sets the throttle value without output (see pulse()).
     *  @param t in in the range [0.0;1.0]
     *  @return true if throttle value is in range; false otherwise.
     */
    bool setThrottle (const float t);
    ///Alias of setThrottle(float)
    bool operator= (const float t);
    
    /** Get the last setted throttle value
     *  @return throttle in range [0.0-1.0].
     */
    float getThrottle () const;
    ///Alias of getThrottle()
    operator float () const;
    
    /** Output the throttle value to the ESC.
     */
    void pulse ();
    ///Alias of pulse()
    void operator() ();
 };

#endif