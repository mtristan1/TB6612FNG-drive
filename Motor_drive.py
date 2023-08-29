from machine import Pin, PWM
from pid import PID
from PWMCounter import PWMCounter
import time, machine

class Motor_drive:
    """
    Motor drive control using TB6612FNG drive IC

    Motor_drive class calls one of the motors "left" and the other "right".
    """
    BOTH = 0
    LEFT = 1
    RIGHT = 2
    TURN_FWD_REV = 0
    TURN_FWD_BRAKE = 1
    AUTO = 1
    MANUAL = 0
    
    def __init__(self,
                 pin_pwm_r,
                 pin_rin1,
                 pin_rin2,
                 pin_pwm_l,
                 pin_lin1,
                 pin_lin2,
                 pwm_freq_hz,
                 pwm_min_duty=0,
                 pwm_max_duty=65535
    ):
        """
        Constructor
        
        Assigns the corresponding pins (PWM, IN1, IN2) to each drive channel. STDBY pin should
        be directly connected high. Pin objects should be defined as OUT before instancing.
        
        :param pwm_max_duty: max PWM value in 16 bit format, represents a 100% duty cycle.
            Using a value lower than 65535 can be useful to lower the overall speed of both motors.
        
        :param pwm_min_duty: min PWM value in 16 bit format, represents a 0% duty cycle.
            Usually motors require a value higher than 0 to start moving.
        """
        # Right channel:
        self.pwm_r = PWM(pin_pwm_r)
        self.pwm_r.freq(pwm_freq_hz)
        self.pwm_r.duty_u16(0)
        self.in1_r = pin_rin1
        self.in2_r = pin_rin2
        # Left channel:
        self.pwm_l = PWM(pin_pwm_l)
        self.pwm_l.freq(pwm_freq_hz)
        self.pwm_l.duty_u16(0)
        self.in1_l = pin_lin1
        self.in2_l = pin_lin2
        # Both channels:
        self.max_pwm = pwm_max_duty
        self.min_pwm = pwm_min_duty
    
    def set_pwm_duty(self, duty_r=None, duty_l=None):
        """
        Sets individual duty value in precentage (0-100) for both pwm channels
        
        :param duty_r (optional): right motor pwm duty % (range: 0-100)
        :param duty_l (optional): left motor pwm duty % (range: 0-100)
        """
        k = (self.max_pwm-self.min_pwm)/100
        if(duty_r != None and 0 <= duty_r <= 100):
            value_u16_r = round(k*duty_r + self.min_pwm)
            self.pwm_r.duty_u16(value_u16_r)
        if(duty_l != None  and 0 <= duty_l <= 100):
            value_u16_l = round(k*duty_l + self.min_pwm)
            self.pwm_l.duty_u16(value_u16_l)
    
    def move_forward(self, pwm_duty=None, which=BOTH):
        """
        Move motors in forward direction, corresponding to CW direction for driver IC
        
        :param pwm_duty (optional): duty value in precentage (0-100) for both pwm channels
        :param which: Selects which motor to forward. Default: self.BOTH, other options: self.LEFT / self.RIGHT
        """
        if(pwm_duty != None):
            self.set_pwm_duty(pwm_duty, pwm_duty)
        if(which in [self.RIGHT, self.BOTH]):
            self.in1_r.on()
            self.in2_r.off()
        if(which in [self.LEFT, self.BOTH]):
            self.in1_l.on()
            self.in2_l.off()
    
    def move_reverse(self, pwm_duty=None, which=BOTH):
        """
        Move motors in reverse direction, corresponding to CCW direction for driver IC
        
        :param pwm_duty (optional): duty value in precentage (0-100) for both pwm channels
        :param which: Selects which motor to reverse. Default: self.BOTH, other options: self.LEFT / self.RIGHT
        """
        if(pwm_duty != None):
            self.set_pwm_duty(pwm_duty, pwm_duty)
        if(which in [self.RIGHT, self.BOTH]):
            self.in1_r.off()
            self.in2_r.on()
        if(which in [self.LEFT, self.BOTH]):
            self.in1_l.off()
            self.in2_l.on()
    
    def turn(self, direction, pwm_duty=None, mode=TURN_FWD_REV):
        """
        Move motors to perform an in-place turn.
        
        :param direction: turn direction, options: self.LEFT / self.RIGHT
        :param pwm_duty (optional): duty value in precentage (0-100) for both pwm channels
        :param mode: turn mode, wheels move in opposite directions, or one is left still.
            Default: self.TURN_FWD_REV, other option: self.TURN_FWD_BRAKE
        """      
        if(direction == self.LEFT):
            # Move right motor forward:
            self.in1_r.on()
            self.in2_r.off()
            if (pwm_duty != None):
                self.set_pwm_duty(duty_r=pwm_duty) 
            if(mode == self.TURN_FWD_BRAKE):
                # Stop left motor:
                self.in1_l.on()
                self.in2_l.on()
                self.set_pwm_duty(duty_l=0)
            elif(mode == self.TURN_FWD_REV):
                # Move left motor reverse:
                self.in1_l.off()
                self.in2_l.on()
                self.set_pwm_duty(duty_l=pwm_duty)
                
        elif(direction == self.RIGHT):
            # Move left motor forward:
            self.in1_l.on()
            self.in2_l.off()
            self.set_pwm_duty(duty_l=pwm_duty)
            if(mode == self.TURN_FWD_BRAKE):
                # Stop right motor:
                self.in1_r.on()
                self.in2_r.on()
                self.set_pwm_duty(duty_r=0)
            elif(mode == self.TURN_FWD_REV):
                # Move right motor reverse:
                self.in1_r.off()
                self.in2_r.on()
                self.set_pwm_duty(duty_r=pwm_duty)
    
    def stop_brake(self, which=BOTH):
        """
        Stops motors using short circuit brake
        
        :param which: selects motor to be stopped. Default: self.BOTH, other options: self.LEFT / self.RIGHT
        """
        if(which in [self.LEFT, self.BOTH]):
            self.in1_l.on()
            self.in2_l.on()
            self.set_pwm_duty(duty_l=0)
        if(which in [self.RIGHT, self.BOTH]):
            self.in1_r.on()
            self.in2_r.on()
            self.set_pwm_duty(duty_r=0)
    
    def stop_coast(self, which=BOTH):
        """
        Stops motors using coast brake
        
        :param which: selects motor to be stopped. Default: self.BOTH, other options: self.LEFT / self.RIGHT
        """
        if(which in [self.LEFT, self.BOTH]):
            self.in1_l.off()
            self.in2_l.off()
            self.set_pwm_duty(duty_l=0)
        if(which in [self.RIGHT, self.BOTH]):
            self.in1_r.off()
            self.in2_r.off()
            self.set_pwm_duty(duty_r=0)

#========================  PID =============================
    
    def init_PID(self, pin_n_enc_l, pin_n_enc_r, timestep, pv_avg, mm_per_pulse):
        """
        Initialize PID control, created PID objects and sets up pins pwm input.
        Should be called only once.
        
        :param pin_n_enc_l: pin gpio number where left motor's encoder signal is connected.
        :param pin_n_enc_r: pin gpio number where right motor's encoder signal is connected.
        :param timestep: time (in seconds) between PID updates.
        :param pv_avg: amount of samples to use for PV moving average window
        :param mm_per_pulse: amount of mm travel distance per encoder pulse
        """
        self._auto_manual = self.AUTO
        self._manual_cv = 0
        self.mm_per_pulse = mm_per_pulse
        
        Pin(pin_n_enc_l, Pin.IN)
        self.enc_l_counter = PWMCounter(pin_n_enc_l, PWMCounter.EDGE_RISING)
        self.enc_l_counter.set_div()
        self.enc_l_counter.start()
        
        Pin(pin_n_enc_r, Pin.IN)
        self.enc_r_counter = PWMCounter(pin_n_enc_r, PWMCounter.EDGE_RISING)
        self.enc_r_counter.set_div()
        self.enc_r_counter.start()
        
        self.pid_timestep = timestep
        self.pv_avg = pv_avg
        self.pv_l_window = [0 for i in range(pv_avg)]
        self.pv_r_window = [0 for i in range(pv_avg)]
        
        self.LeftPID = PID(output_limits=(-100,100), sample_time=None)
        self.RightPID = PID(output_limits=(-100,100), sample_time=None)
        self.reset_enc_count()
        self._last_cv_L = 0
        self._last_cv_R = 0
        self._last_pv_L = 0
        self._last_pv_R = 0
    
    def reset_enc_count(self):
        """
        Reset encoders pulse counters and PV moving average samples
        """
        self.enc_l_counter.reset()
        self.enc_r_counter.reset()
        self.enc_l_count = 0
        self.enc_r_count = 0
        self._last_time = -1
        self.pv_l_window = [0 for i in range(self.pv_avg)]
        self.pv_r_window = [0 for i in range(self.pv_avg)]
    
    def get_enc_count(self, mean=True):
        """
        Returns current encoder count
        
        :param mean (optional): if True, returns the mean value of both channels.
            If False, returns a list with each channel's [left, right] count.
        """
        if mean:
            return (self.enc_l_count + self.enc_r_count)/2
        else:
            return [self.enc_l_count, self.enc_r_count]
    
    def set_pid_sp(self, left_sp, right_sp):
        """
        Sets setpoint for PID controllers

        :param left_sp: Setpoint for left motor's PID
        :param right_sp: Setpoint for right motor's PID
        """
        self.LeftPID.setpoint = left_sp
        self.RightPID.setpoint = right_sp
    
    def reset_pid(self):
        """
        Resets both PID controllers (set internal terms to 0)
        """
        self.LeftPID.reset()
        self.RightPID.reset()
    
    def set_pid_tuning(self, tunings):
        """
        Sets PID tunings

        :param tunings: tuple containing the three PID terms in this ordrer -> (Kp, Ki, Kd).
        """
        self.LeftPID.tunings = tunings
        self.RightPID.tunings = tunings
    
    def set_pid_mode(self, auto_manual, cv=0):
        """
        Switch both PID controllers to manual or auto mode
        
        :param auto_manual: Enables manual mode (self.MANUAL) or auto mode (self.AUTO)
        :param cv: CV for manual mode (ignored if enabled == False)
        """
        if auto_manual == self.AUTO:
            self._auto_manual = self.AUTO
        elif auto_manual == self.MANUAL:
            self._auto_manual = self.MANUAL
            self._manual_cv = cv
    
    def update_PID(self, bias_l=0.0, bias_r=0.0):
        """
        Updates PID (gets CV values from current PV values). This method must be called periodacally (DO NOT call it
        within an interrupt, since it needs to allocate memory to work and may rise a memory error exception).
        
        :param bias_l (optional): bias value used for feedforward control on the left motor.
            This value will be added directly to the calculated CV output.
        :param bias_r (optional): idem for right motor.
        """
        # Read data:
        state = machine.disable_irq()
        time_now = time.ticks_us()
        l_count = self.enc_l_counter.read()
        r_count = self.enc_r_counter.read()
        machine.enable_irq(state)
        if self._last_time == -1:
            timestep = self.pid_timestep
        else:
            timestep = time.ticks_diff(time_now, self._last_time) / 1e6
        # Filters PV with moving avg:
        self.pv_l_window.pop(0)
        self.pv_l_window.append(l_count-self.enc_l_count)
        self.pv_r_window.pop(0)
        self.pv_r_window.append(r_count-self.enc_r_count)
        pv_L = sum(self.pv_l_window)/self.pv_avg *self.mm_per_pulse / timestep
        pv_R = sum(self.pv_r_window)/self.pv_avg *self.mm_per_pulse / timestep
        self.enc_l_count = l_count
        self.enc_r_count = r_count
        self._last_time = time_now
        # Update output:
        if self._auto_manual == self.AUTO:
            cv_L = self.LeftPID(pv_L, dt=timestep) +bias_l
            cv_R = self.RightPID(pv_R, dt=timestep) +bias_r
        elif self._auto_manual == self.MANUAL:
            cv_L = self._manual_cv
            cv_R = self._manual_cv
        self.set_pwm_duty(duty_l=abs(cv_L), duty_r=abs(cv_R))
        self._last_cv_L = cv_L
        self._last_cv_R = cv_R
        self._last_pv_L = pv_L
        self._last_pv_R = pv_R
    
    def get_PID_status(self):
        """
        Returns a string with information about the PID controllers status.
        """
        elements = [self.enc_l_count,
                    self.LeftPID.setpoint,
                    self._last_pv_L,
                    self._last_cv_L,
                    self.LeftPID.components,
                    self.enc_r_count,
                    self.RightPID.setpoint,
                    self._last_pv_R,
                    self._last_cv_R,
                    self.RightPID.components]
        info_str = ""
        for n in elements:
            if isinstance(n, tuple):
                info_str += "k:" + '{:.1f}'.format(n[0])
                info_str += " i:" + '{:.1f}'.format(n[0])
                info_str += " d:" + '{:.1f}'.format(n[0]) + "\t"
            else:
                info_str += '{:.2f}'.format(n) + "\t"
        return(info_str)