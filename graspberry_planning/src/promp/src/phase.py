import numpy as np

class PhaseGenerator():

    def __init__(self):
        return

    def phase(self, time):

        # Base class...
        return



class LinearPhaseGenerator(PhaseGenerator): # object called PhaseGenerator is an instance of the class, class X and class X(object) are no different for Python

    def __init__(self, phaseVelocity = 1.0): # __init__() is a method defining initial states to an instance of the class called self, phase velocity is z_dot in paper where z is the phase variable; change the phase velocity to modulate the speed of the movement

        PhaseGenerator.__init__(self)
        self.phaseVelocity = phaseVelocity


    def phase(self, time):

        phase = np.array(time) * self.phaseVelocity  # phase = z = z_dot = z/t --> z = z_dot * time 

        return phase


class RhythmicPhaseGenerator(PhaseGenerator):

    def __init__(self, phasePeriod = 1.0, useModulo = False ):
        PhaseGenerator.__init__(self)
        self.phasePeriod = phasePeriod
        self.useModulo = useModulo

    def phase(self, time):

        phase = time / self.phasePeriod
        if (self.useModulo):
            phase = np.mod(phase, 1.0)

        return phase



class ExpDecayPhaseGenerator(PhaseGenerator):

    def __init__(self, duration = 1, alphaPhase = 2):

        PhaseGenerator.__init__(self)

        self.tau =  1.0 / duration
        self.alphaPhase = alphaPhase

    def phase(self, time):

        time = time * self.tau

        phase = np.exp(- time * self.alphaPhase)

        return phase

if __name__ == "__main__":

    import matplotlib.pyplot as plt
    import numpy as np

    phaseGenerator = ExpDecayPhaseGenerator()
    time = np.linspace(0,1, 100)
    phase =  phaseGenerator.phase(time)

    plt.figure()
    plt.plot(time, phase)
    plt.hold(True)

    phaseGenerator.tau = 2
    phase = phaseGenerator.phase(time)
    plt.plot(time, phase)

    phaseGenerator.tau = 0.5
    phase = phaseGenerator.phase(time)
    plt.plot(time, phase)

    plt.show()


    # phaseGenerator_ryth = RhythmicPhaseGenerator(PhaseGenerator)
    
    # phase =  phaseGenerator_ryth.phase(time)

    # plt.figure()
    # plt.plot(time, phase)

    # phaseGenerator_lin = LinearPhaseGenerator(PhaseGenerator)
    
    # phase =  phaseGenerator_lin.phase(time)

    # plt.figure()
    # plt.plot(time, phase)
    # plt.show()

    print('PhaseGeneration Done')

