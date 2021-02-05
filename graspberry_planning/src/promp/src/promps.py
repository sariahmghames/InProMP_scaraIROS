import matplotlib.pyplot as plt
import numpy as np
import phase as phase
import basis as basis

import scipy.stats as stats


class ProMP:

    def __init__(self, basis, phase, numDoF):
        # self.basis = basis
        # self.phase = phase
        # self.numDoF = numDoF
        # self.numBasis = basis.numBasis
        # self.numWeights = basis.numBasis * self.numDoF  # for 1 demo
        # self.mu = np.zeros(self.numWeights) # mu of w has size = w, eq (4) in Geri paper
        # self.covMat = np.eye(self.numWeights) # covMat of w
        # self.observationSigma = np.ones(self.numDoF) # variance of observations (y), or sigma y (noise:epsilon_y variance)
        self.basis = basis
        self.phase = phase
        self.numDoF = numDoF
        self.numBasis = basis.numBasis
        self.numWeights = basis.numBasis * self.numDoF  # for 1 demo
        self.muX = np.zeros(self.numWeights) # mu of w has size = w, eq (4) in Geri paper
        self.covMatX = np.eye(self.numWeights) # covMat of w
        self.muY = np.zeros(self.numWeights) # mu of w has size = w, eq (4) in Geri paper
        self.covMatY = np.eye(self.numWeights) # covMat of w
        self.muZ = np.zeros(self.numWeights) # mu of w has size = w, eq (4) in Geri paper
        self.covMatZ = np.eye(self.numWeights) # covMat of w
        self.observationSigma = np.ones(self.numDoF) # variance of observations (y), or sigma y (noise:epsilon_y variance)


    # def getTrajectorySamples(self, time, n_samples=1):
    #     basisMultiDoF = self.basis.basisMultiDoF(time, self.numDoF)
    #     weights = np.random.multivariate_normal(self.mu, self.covMat, n_samples)
    #     weights = weights.transpose()
    #     trajectoryFlat = basisMultiDoF.dot(weights)
    #     # a = trajectoryFlat
    #     trajectoryFlat = trajectoryFlat.reshape((self.numDoF, trajectoryFlat.shape[0] / self.numDoF, n_samples))
    #     trajectoryFlat = np.transpose(trajectoryFlat, (1, 0, 2))
    #     # trajectoryFlat = trajectoryFlat.reshape((a.shape[0] / self.numDoF, self.numDoF, n_samples))

    #     return trajectoryFlat


    def getTrajectorySamplesX(self, time, n_samples=1):
        basisMultiDoF = self.basis.basisMultiDoF(time, self.numDoF)
        weightsX = np.random.multivariate_normal(self.muX, self.covMatX, n_samples)
        weightsX = weightsX.transpose()
        trajectoryFlatX = basisMultiDoF.dot(weightsX)
        trajectoryFlatX = trajectoryFlatX.reshape((self.numDoF, trajectoryFlatX.shape[0] / self.numDoF, n_samples))
        trajectoryFlatX = np.transpose(trajectoryFlatX, (1, 0, 2))
        return trajectoryFlatX


    def getTrajectorySamplesY(self, time, n_samples=1):
        basisMultiDoF = self.basis.basisMultiDoF(time, self.numDoF)
        weightsY = np.random.multivariate_normal(self.muY, self.covMatY, n_samples)
        weightsY = weightsY.transpose()
        trajectoryFlatY = basisMultiDoF.dot(weightsY)
        trajectoryFlatY = trajectoryFlatY.reshape((self.numDoF, trajectoryFlatY.shape[0] / self.numDoF, n_samples))
        trajectoryFlatY = np.transpose(trajectoryFlatY, (1, 0, 2))
        return trajectoryFlatY


    def getTrajectorySamplesZ(self, time, n_samples=1):
        basisMultiDoF = self.basis.basisMultiDoF(time, self.numDoF)
        weightsZ = np.random.multivariate_normal(self.muZ, self.covMatZ, n_samples)
        weightsZ = weightsZ.transpose()
        trajectoryFlatZ = basisMultiDoF.dot(weightsZ)
        trajectoryFlatZ = trajectoryFlatZ.reshape((self.numDoF, trajectoryFlatZ.shape[0] / self.numDoF, n_samples))
        trajectoryFlatZ = np.transpose(trajectoryFlatZ, (1, 0, 2))
        return trajectoryFlatZ


    def getMeanAndCovarianceTrajectory(self, time): # the time at which we want to condition, here is onlt t = 1s
        basisMultiDoF = self.basis.basisMultiDoF(time, self.numDoF) # generates phi matrix for the whole DoFs
        trajectoryFlat = basisMultiDoF.dot(self.mu.transpose()) # traj flat here is the mean of the traj (mean of the ProMP) , mu = nb of weights = 15x1 -- > transpose = 1x15
        trajectoryMean = trajectoryFlat.reshape((self.numDoF, trajectoryFlat.shape[0] / self.numDoF)) # proMP mean for 3 dofs at t 1s
        trajectoryMean = np.transpose(trajectoryMean, (1, 0)) # axis = (1, 0)
        covarianceTrajectory = np.zeros((self.numDoF, self.numDoF, len(time))) # covarianceTrajectory is a 3d array , sigma yt, initialization, here len(time)= 1 as we are interested in time = 1s only

        for i in range(len(time)):
            basisSingleT = basisMultiDoF[slice(i, (self.numDoF - 1) * len(time) + i + 1, len(time)), :]
            covarianceTimeStep = basisSingleT.dot(self.covMat).dot(basisSingleT.transpose()) # equiv to phi_t * sigma omega * phi_t ^T (eq 4), no sigma_y ?
            covarianceTrajectory[:, :, i] = covarianceTimeStep

        return trajectoryMean, covarianceTrajectory

    def getMeanAndStdTrajectory(self, time):
        basisMultiDoF = self.basis.basisMultiDoF(time, self.numDoF)
        trajectoryFlat = basisMultiDoF.dot(self.mu.transpose())
        trajectoryMean = trajectoryFlat.reshape((self.numDoF, trajectoryFlat.shape[0] / self.numDoF))
        trajectoryMean = np.transpose(trajectoryMean, (1, 0))
        stdTrajectory = np.zeros((len(time), self.numDoF))

        for i in range(len(time)):
            basisSingleT = basisMultiDoF[slice(i, (self.numDoF - 1) * len(time) + i + 1, len(time)), :] # basis at time step t = T
            covarianceTimeStep = basisSingleT.dot(self.covMat).dot(basisSingleT.transpose())
            stdTrajectory[i, :] = np.sqrt(np.diag(covarianceTimeStep))

        return trajectoryMean, stdTrajectory

    
    def getMeanAndCovarianceTrajectoryFull(self, time):
        basisMultiDoF = self.basis.basisMultiDoF(time, self.numDoF)

        meanFlat = basisMultiDoF.dot(self.mu.transpose())
        covarianceTrajectory = basisMultiDoF.dot(self.covMat).dot(basisMultiDoF.transpose())

        return meanFlat, covarianceTrajectory

    
    def jointSpaceConditioning(self, time, desiredTheta, desiredVar):
        newProMP = ProMP(self.basis, self.phase, self.numDoF)
        basisMatrix = self.basis.basisMultiDoF(time, self.numDoF)
        #print('basisMatrix=', basisMatrix)
        temp = self.covMat.dot(basisMatrix.transpose())
        #print('learnt covMat=', self.covMat)
        #print('learnt covMat=', self.covMat.shape)
        #print('basisMatrix=', basisMatrix.shape)
        L = np.linalg.solve(desiredVar + basisMatrix.dot(temp), temp.transpose())
        L = L.transpose()
        newProMP.mu = self.mu + L.dot(desiredTheta - basisMatrix.dot(self.mu))
        newProMP.covMat = self.covMat - L.dot(basisMatrix).dot(self.covMat)
        return newProMP


    def taskSpaceConditioning(self, time, desiredXmean, desiredXVar):
        newProMP = ProMP(self.basis, self.phase, 1)
        basisMatrix = self.basis.basisMultiDoF(time, 1)
        temp = self.covMat.dot(basisMatrix.transpose())
        L = np.linalg.solve(desiredXVar + basisMatrix.dot(temp), temp.transpose())
        L = L.transpose()
        newProMP.mu = self.mu + L.dot(desiredXmean - basisMatrix.dot(self.mu))
        newProMP.covMat = self.covMat - L.dot(basisMatrix).dot(self.covMat)
        return newProMP


    def taskSpaceConditioning_Sariah(self, time, desiredXmean, desiredXVar):
        newProMP = ProMP(self.basis, self.phase, 1)
        basisMatrix = self.basis.basisMultiDoF(time, 1)
        tempX = self.covMatX.dot(basisMatrix.transpose())
        tempY = self.covMatY.dot(basisMatrix.transpose())
        tempZ = self.covMatZ.dot(basisMatrix.transpose())
        LX = np.linalg.solve(desiredXVar[0,0] + basisMatrix.dot(tempX), tempX.transpose())
        LX = LX.transpose()
        LY = np.linalg.solve(desiredXVar[1,1] + basisMatrix.dot(tempY), tempY.transpose())
        LY = LY.transpose()
        LZ = np.linalg.solve(desiredXVar[2,2] + basisMatrix.dot(tempZ), tempZ.transpose())
        LZ = LZ.transpose()
        newProMP.muX = self.muX + LX.dot(desiredXmean[0] - basisMatrix.dot(self.muX))
        newProMP.covMatX = self.covMatX - LY.dot(basisMatrix).dot(self.covMatX)
        newProMP.muY = self.muY + LY.dot(desiredXmean[1] - basisMatrix.dot(self.muY))
        newProMP.covMatY = self.covMatY - LY.dot(basisMatrix).dot(self.covMatY)
        newProMP.muZ = self.muZ + LZ.dot(desiredXmean[2] - basisMatrix.dot(self.muZ))
        newProMP.covMatZ = self.covMatZ - LZ.dot(basisMatrix).dot(self.covMatZ)
        return newProMP


    def getTrajectoryLogLikelihood(self, time, trajectory):

        trajectoryFlat = trajectory.transpose().reshape(trajectory.shape[0] * trajectory.shape[1])
        meanFlat, covarianceTrajectory = self.getMeanAndCovarianceTrajectoryFull(self, time)

        return stats.multivariate_normal.logpdf(trajectoryFlat, mean=meanFlat, cov=covarianceTrajectory)

    def getWeightsLogLikelihood(self, weights):

        return stats.multivariate_normal.logpdf(weights, mean=self.mu, cov=self.covMat)

    def plotProMP(self, time, indices=None):
        import plotter as plotter

        trajectoryMean, stdTrajectory = self.getMeanAndStdTrajectory(time)

        plotter.plotMeanAndStd(time, trajectoryMean, stdTrajectory, indices)


# class newProMP():
#     def __init__(self, proMP, basisGen, numDoF, numBasis ):
#         self.proMP = proMP
#         self.basis = basisGen
#         self.numDoF = numDoF
#         self.numBasis = numBasis
#         self.numWeights = numBasis * self.numDoF  # for 1 demo
#         self.muX = np.zeros(self.numWeights) # mu of w has size = w, eq (4) in Geri paper
#         self.covMatX = np.eye(self.numWeights) # covMat of w
#         self.muY = np.zeros(self.numWeights) # mu of w has size = w, eq (4) in Geri paper
#         self.covMatY = np.eye(self.numWeights) # covMat of w
#         self.muZ = np.zeros(self.numWeights) # mu of w has size = w, eq (4) in Geri paper
#         self.covMatZ = np.eye(self.numWeights) # covMat of w

#     def taskSpaceConditioning_Sariah(self, time, desiredXmean, desiredXVar):
#         #self.basis = basisGen
#         newPMP = newProMP(self.basis, self.proMP.phase, self.numDoF, self.numBasis)
#         basisMatrix = self.basis.basisMultiDoF(time, 1)
#         tempX = self.proMP.covMatX.dot(basisMatrix.transpose()) # not aligned dim when using different nb of basis for each conditioned promp
#         tempY = self.proMP.covMatY.dot(basisMatrix.transpose())
#         tempZ = self.proMP.covMatZ.dot(basisMatrix.transpose())
#         LX = np.linalg.solve(desiredXVar[0,0] + basisMatrix.dot(tempX), tempX.transpose())
#         LX = LX.transpose()
#         LY = np.linalg.solve(desiredXVar[1,1] + basisMatrix.dot(tempY), tempY.transpose())
#         LY = LY.transpose()
#         LZ = np.linalg.solve(desiredXVar[2,2] + basisMatrix.dot(tempZ), tempZ.transpose())
#         LZ = LZ.transpose()
#         newPMP.muX = self.proMP.muX + LX.dot(desiredXmean[0] - basisMatrix.dot(self.proMP.muX))
#         newPMP.covMatX = self.proMP.covMatX - LY.dot(basisMatrix).dot(self.proMP.covMatX)
#         newPMP.muY = self.proMP.muY + LY.dot(desiredXmean[1] - basisMatrix.dot(self.proMP.muY))
#         newPMP.covMatY = self.proMP.covMatY - LY.dot(basisMatrix).dot(self.proMP.covMatY)
#         newPMP.muZ = self.proMP.muZ + LZ.dot(desiredXmean[2] - basisMatrix.dot(self.proMP.muZ))
#         newPMP.covMatZ = self.proMP.covMatZ - LZ.dot(basisMatrix).dot(self.proMP.covMatZ)
#         return newPMP




class MAPWeightLearner(): # learn omega parameter by MAP/ML, refer eq 3.28 in sequential learning techniques known as regularized Least square solution for a minimization problem 

    def __init__(self, proMP, regularizationCoeff=10 ** -9, priorCovariance=10 ** -4, priorWeight=1):
        self.proMP = proMP
        self.priorCovariance = priorCovariance # sigma omega new
        self.priorWeight = priorWeight
        self.regularizationCoeff = regularizationCoeff

    def learnFromData(self, trajectoryList, timeList):
        numTraj = len(trajectoryList)  # number of demos 
        #print('numTraj=', numTraj)
        weightMatrix = np.zeros((numTraj, self.proMP.numWeights)) # 121 (nb of demos) x (5*3)
        for i in range(numTraj):
            trajectory = trajectoryList[i]  # pick 1 demo will have a shape 162 samples x 3dofs
            time = timeList[i] 
            #print('trajectory shape=', trajectory.shape)
            trajectoryFlat = trajectory.transpose().reshape(trajectory.shape[0] * trajectory.shape[1]) # trajectory.transpose() = 3x162 , with reshape we get a row of samples among all dofs
            #print('trajectoryflat=', trajectoryFlat.shape)
            basisMatrix = self.proMP.basis.basisMultiDoF(time, self.proMP.numDoF)   # can it access basisMultiDoF() ? get the values of basis function at different time instant for multi dofs from single DoF , so matrix phi
            temp = basisMatrix.transpose().dot(basisMatrix) + np.eye(self.proMP.numWeights) * self.regularizationCoeff # self.proMP.numWeights = basisSingleDoF.shape[0] * numDOF
            #print('basis 1 demo=',basisMatrix.shape)
            weightVector = np.linalg.solve(temp, basisMatrix.transpose().dot(trajectoryFlat))
            weightMatrix[i, :] = weightVector

        self.proMP.mu = np.mean(weightMatrix, axis=0) # mean of weights from prior , along rows
        #print('weights of learnt ProMP=', weightMatrix)
        #print('mean of learnt ProMP=', self.proMP.mu)
        all_zeros = not np.any(weightMatrix)
        print('all zeros=', all_zeros)
        sampleCov = np.cov(weightMatrix.transpose())
        self.proMP.covMat = (numTraj * sampleCov + self.priorCovariance * np.eye(self.proMP.numWeights)) / (
                    numTraj + self.priorCovariance) # cov of weight matrix, from prior .. ?
        
        ## Trial Sariah to get here the mean of the learnt proMP
        # print('weight matrix =', weightMatrix.shape)
        # print('weight matrix mean =', (self.proMP.mu).shape)
        # trajectoryFlat = basisMatrix.dot(self.proMP.mu)

        # trajectoryFlat = trajectoryFlat.reshape((self.proMP.numDoF, trajectoryFlat.shape[0] / self.proMP.numDoF))
        # print(' trajectoryFlat =', trajectoryFlat.shape)
        # trajectoryFlat = np.transpose(trajectoryFlat, (1, 0))

        # return trajectoryFlat

    def learnFromData1(self, trajectoryList, timeList):
        numTraj = len(trajectoryList)  # number of demos 
        weightMatrix = np.zeros((numTraj, self.proMP.numWeights)) # 121 (nb of demos) x (5*3)
        for i in range(numTraj):
            trajectory = trajectoryList[i]  # pick 1 demo will have a shape 162 samples x 3dofs
            time = timeList[i] 
            print('trajectory shape=', trajectory.shape)
            trajectoryFlat = trajectory.transpose() # trajectory.transpose() = 3x162 , with reshape we get a row of samples among all dofs
            #print('trajectoryflat=', trajectoryFlat.shape)
            basisMatrix = self.proMP.basis.basisMultiDoF(time, 1)   # can it access basisMultiDoF() ? get the values of basis function at different time instant for multi dofs from single DoF , so matrix phi
            temp = basisMatrix.transpose().dot(basisMatrix) + np.eye(self.proMP.numBasis) * self.regularizationCoeff # self.proMP.numWeights = basisSingleDoF.shape[0] * numDOF
            print('basis 1 demo and 1 joint=',basisMatrix.shape)
            weightVector = np.linalg.solve(temp, basisMatrix.transpose().dot(trajectoryFlat))

        self.proMP.mu = np.mean(weightMatrix, axis=0) # mean of weights from prior , along rows

        sampleCov = np.cov(weightMatrix.transpose())
        self.proMP.covMat = (numTraj * sampleCov + self.priorCovariance * np.eye(self.proMP.numBasis)) / (
                    numTraj + self.priorCovariance) # cov of weight matrix, from prior .. ?
        trajectoryFlat = basisMatrix.dot(self.proMP.mu)
        return trajectoryFlat

    def learnFromXDataTaskSapce(self, trajectoryList, timeList):  # Added by Sariah
        numTraj = len(trajectoryList)  # number of demos 
        print('numTraj=', numTraj)
        weightMatrix = np.zeros((numTraj, self.proMP.numWeights)) # 121 (nb of demos) x (15*1)
        for i in range(numTraj):
            trajectory = trajectoryList[i]  # pick 1 demo will have a shape 162 samples x 3dofs
            time = timeList[i] 
            trajectoryFlat = trajectory.transpose() # trajectory.transpose() = 3x162 , with reshape we get a row of samples among all dofs
            basisMatrix = self.proMP.basis.basisMultiDoF(time, 1)   # can it access basisMultiDoF() ? get the values of basis function at different time instant for multi dofs from single DoF , so matrix phi
            temp = basisMatrix.transpose().dot(basisMatrix) + np.eye(self.proMP.numWeights) * self.regularizationCoeff # self.proMP.numWeights = basisSingleDoF.shape[0] * numDOF
            weightVector = np.linalg.solve(temp, basisMatrix.transpose().dot(trajectory))
            #print('weight vector size=', weightVector.shape)
            weightMatrix[i, :] = np.transpose(weightVector)  # from each demo learn a weight vector for a dim in task space

        self.proMP.muX = np.mean(weightMatrix, axis=0) # mean of weights from prior , along rows

        sampleCov = np.cov(weightMatrix.transpose())
        self.proMP.covMatX = (numTraj * sampleCov + self.priorCovariance * np.eye(self.proMP.numWeights)) / (
                    numTraj + self.priorCovariance) # cov of weight matrix, from prior .. ?

    def learnFromYDataTaskSapce(self, trajectoryList, timeList):  # Added by Sariah
        numTraj = len(trajectoryList)  # number of demos 
        print('numTraj=', numTraj)
        weightMatrix = np.zeros((numTraj, self.proMP.numWeights)) # 121 (nb of demos) x (15*1)
        for i in range(numTraj):
            trajectory = trajectoryList[i]  # pick 1 demo will have a shape 162 samples x 3dofs
            time = timeList[i] 
            trajectoryFlat = trajectory.transpose() # trajectory.transpose() = 3x162 , with reshape we get a row of samples among all dofs
            basisMatrix = self.proMP.basis.basisMultiDoF(time, 1)   # can it access basisMultiDoF() ? get the values of basis function at different time instant for multi dofs from single DoF , so matrix phi
            temp = basisMatrix.transpose().dot(basisMatrix) + np.eye(self.proMP.numWeights) * self.regularizationCoeff # self.proMP.numWeights = basisSingleDoF.shape[0] * numDOF
            weightVector = np.linalg.solve(temp, basisMatrix.transpose().dot(trajectory))
            #print('weight vector size=', weightVector.shape)
            weightMatrix[i, :] = np.transpose(weightVector)  # from each demo learn a weight vector for a dim in task space

        self.proMP.muY = np.mean(weightMatrix, axis=0) # mean of weights from prior , along rows

        sampleCov = np.cov(weightMatrix.transpose())
        self.proMP.covMatY = (numTraj * sampleCov + self.priorCovariance * np.eye(self.proMP.numWeights)) / (
                    numTraj + self.priorCovariance) # cov of weight matrix, from prior .. ?


    def learnFromZDataTaskSapce(self, trajectoryList, timeList):  # Added by Sariah
        numTraj = len(trajectoryList)  # number of demos 
        print('numTraj=', numTraj)
        weightMatrix = np.zeros((numTraj, self.proMP.numWeights)) # 121 (nb of demos) x (15*1)
        for i in range(numTraj):
            trajectory = trajectoryList[i]  # pick 1 demo will have a shape 162 samples x 3dofs
            time = timeList[i] 
            trajectoryFlat = trajectory.transpose() # trajectory.transpose() = 3x162 , with reshape we get a row of samples among all dofs
            basisMatrix = self.proMP.basis.basisMultiDoF(time, 1)   # can it access basisMultiDoF() ? get the values of basis function at different time instant for multi dofs from single DoF , so matrix phi
            temp = basisMatrix.transpose().dot(basisMatrix) + np.eye(self.proMP.numWeights) * self.regularizationCoeff # self.proMP.numWeights = basisSingleDoF.shape[0] * numDOF
            weightVector = np.linalg.solve(temp, basisMatrix.transpose().dot(trajectory))
            #print('weight vector size=', weightVector.shape)
            weightMatrix[i, :] = np.transpose(weightVector)  # from each demo learn a weight vector for a dim in task space

        self.proMP.muZ = np.mean(weightMatrix, axis=0) # mean of weights from prior , along rows

        sampleCov = np.cov(weightMatrix.transpose())
        self.proMP.covMatZ = (numTraj * sampleCov + self.priorCovariance * np.eye(self.proMP.numWeights)) / (
                    numTraj + self.priorCovariance) # cov of weight matrix, from prior .. ?




if __name__ == "__main__":

    phaseGenerator = phase.LinearPhaseGenerator()
    basisGenerator = basis.NormalizedRBFBasisGenerator(phaseGenerator, numBasis=5, duration=10, basisBandWidthFactor=5,
                                                       numBasisOutside=1)
    time = np.linspace(0, 1, 100)
    nDof = 3




    proMP = ProMP(basisGenerator, phaseGenerator, nDof)  # 3 argument = nDOF
    trajectories = proMP.getTrajectorySamples(time, 4)  # 2nd argument is numSamples/Demonstrations/trajectories
    meanTraj, covTraj = proMP.getMeanAndCovarianceTrajectory(time)
    plotDof = 2
    # plt.figure()
    # plt.plot(time, trajectories1[:, plotDof, :])
    # #
    # plt.figure()
    # plt.plot(time, meanTraj[:, 0])

    learnedProMP = ProMP(basisGenerator, phaseGenerator, nDof)
    learner = MAPWeightLearner(learnedProMP)
    trajectoriesList = []
    timeList = []

    for i in range(trajectories.shape[2]):
        trajectoriesList.append(trajectories[:, :, i])
        timeList.append(time)

    learner.learnFromData(trajectoriesList, timeList)
    trajectories = learnedProMP.getTrajectorySamples(time, 10)
    plt.figure()
    plt.plot(time, trajectories[:, plotDof, :])
    plt.xlabel('time')
    plt.title('MAP sampling')
    ################################################################

    # Conditioning in JointSpace
    desiredTheta = np.array([0.5, 0.7, 0.9, 0.2, 0.6, 0.8, 0.1])
    desiredVar = np.eye(len(desiredTheta)) * 0.0001
    newProMP = proMP.jointSpaceConditioning(0.5, desiredTheta=desiredTheta, desiredVar=desiredVar)
    trajectories = newProMP.getTrajectorySamples(time, 4)
    plt.figure()
    plt.plot(time, trajectories[:, plotDof, :])
    plt.xlabel('time')
    plt.title('Joint-Space conditioning')
    newProMP.plotProMP(time, [3, 4])

    plt.show()
