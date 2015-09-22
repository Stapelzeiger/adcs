% L = I * w
% I : moment of inertia = sum(r^2 * m) [m^2*kg]

classdef RotationBody2D < handle
    properties (SetAccess = private)
        inertia = 1 % moment of inertia [m^2*kg]
        velocity = 0 % angular velocity [rad/s]
        position = 0 % angular position [rad]
    end

    methods
        function pos = getPos(self)
            pos = self.position;
        end

        function vel = getVel(self)
            vel = self.velocity;
        end

        function setVel(self, vel)
            self.velocity = vel;
        end

        function setPos(self, pos)
            self.position = pos;
        end

        function setInertia(self, inertia)
            assert(inertia > 0);
            self.inertia = inertia;
        end

        function inertia = getInertia(self)
            inertia = self.inertia;
        end

<<<<<<< Updated upstream
        function posVel = measurePosVel(self, noiseStdDev)
            noise = [randn()*noiseStdDev(1); randn()*noiseStdDev(2)];
            posVel = [self.position; self.velocity] + noise;
        end

        function applyTorque(self, torque, duration)
=======
        function update(self, torque, duration)
>>>>>>> Stashed changes
            acc = torque / self.inertia;
            self.position = self.position + self.velocity * duration;
            self.position = self.position + 1/2*acc * duration^2;
            self.velocity = self.velocity + duration * acc;
        end
    end
end
