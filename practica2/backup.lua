-- Generate a sample from a Gaussian distribution
function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) *
            math.cos(2 * math.pi * math.random()) + mean
end

function dispersionRange(rangeList)
    minPoint = math.huge
    maxPoint = -math.huge
    for i, value in ipairs(rangeList) do
        if(value < minPoint) then
            minPoint = value
        end
        
        if(value > maxPoint) then
            maxPoint = value
        end
    end
    return {minPoint, maxPoint}
end

function systematicError(rangeList)
    total = 0
    for i,value in ipairs(rangeList) do
        total = total + value
    end
    return total / table.getn(rangeList)
end

function covarianzaMatrix(xList, yList, xyAverage)
    sum_00 = 0
    sum_01 = 0
    sum_10 = 0
    sum_11 = 0
    
    for counter = 1, #xList do
        -- sumatory in 00
        sum_00 = sum_00 + math.pow((xList[counter] - xyAverage[1]),2)
        sum_01 = sum_01 + ( (xList[counter] - xyAverage[1]) * (yList[counter] - xyAverage[2]) )
        sum_10 = sum_10 + ( (xList[counter] - xyAverage[1]) * (yList[counter] - xyAverage[2]) )
        sum_11 = sum_11 + math.pow( (yList[counter] - xyAverage[2]) , 2)
    end
    
    listSize = table.getn(xList)
    
    return { {sum_00/listSize, sum_01/listSize} , { sum_10/listSize, sum_11/listSize }  }
end

-- Return robot to a location
function resetBase(handle, matrix)
    allModelObjects = sim.getObjectsInTree(handle) -- get all objects in the model
    sim.setThreadAutomaticSwitch(false)
    for i=1,#allModelObjects,1 do
        sim.resetDynamicObject(allModelObjects[i]) -- reset all objects in the model
    end
    sim.setObjectMatrix(handle,-1,matrix)
    sim.setThreadAutomaticSwitch(true)
end

function createRandomBumpyFloor()
    print ("Generating new random bumpy floor.")
    sim.setThreadAutomaticSwitch(false)

    -- Remove existing bumpy floor if there already is one
    if (heightField ~= nil) then
        sim.setObjectPosition(heightField, heightField, {0.05, 0, 0})
        return
    end
    --  Create random bumpy floor for robot to drive on
    floorSize = 5
    --heightFieldResolution = 0.3
    --heightFieldNoise = 0.00000005
    heightFieldResolution = 0.1
    heightFieldNoise = 0.0000008
    cellsPerSide = floorSize / heightFieldResolution
    cellHeights = {}
    for i=1,cellsPerSide*cellsPerSide,1 do
        table.insert(cellHeights, gaussian(0, heightFieldNoise))
    end
    heightField=sim.createHeightfieldShape(0, 0, cellsPerSide, cellsPerSide, floorSize, cellHeights)
    -- Make the floor invisible
    sim.setObjectInt32Param(heightField,10,0)
    sim.setThreadAutomaticSwitch(true)
end


-- This function is executed exactly once when the scene is initialised
function sysCall_init()

    tt = sim.getSimulationTime()
    print("Init hello", tt)
          
    robotBase=sim.getObjectHandle(sim.handle_self) -- robot handle
    leftMotor=sim.getObjectHandle("leftMotor") -- Handle of the left motor
    rightMotor=sim.getObjectHandle("rightMotor") -- Handle of the right motor
    turretMotor=sim.getObjectHandle("turretMotor") -- Handle of the turret motor
    turretSensor=sim.getObjectHandle("turretSensor")
    
    -- We only update graphs every few steps because the simulation slows down otherwise
    UPDATE_GRAPHS_EVERY = 20
    graphSteps = 0

    motorGraph=sim.getObjectHandle("motorGraph")
    trajectoryGraph=sim.getObjectHandle("trajectoryGraph")
    
 

    -- Create bumpy floor for robot to drive on
    createRandomBumpyFloor()

    -- Save robot start position so we can return it there later
    robotStartMatrix=sim.getObjectMatrix(robotBase,-1)
   
    -- Usual rotation rate for wheels (radians per second)
    speedBase = 5
    
    -- Which step are we in? 
    -- 0 is a dummy value which is immediately completed
    stepCounter = 0
    
    stepList = {}
        
    table.insert(stepList, {"forward", 1.00})
    table.insert(stepList, {"stop"})
    table.insert(stepList, {"turn", math.rad(90)})
    table.insert(stepList, {"stop"})
    table.insert(stepList, {"forward", 1.00})
    table.insert(stepList, {"stop"})
    table.insert(stepList, {"turn", math.rad(90)})
    table.insert(stepList, {"stop"})
    table.insert(stepList, {"forward", 1.00})
    table.insert(stepList, {"stop"})
    table.insert(stepList, {"turn", math.rad(90)})
    table.insert(stepList, {"stop"})
    table.insert(stepList, {"forward", 1.00})
    table.insert(stepList, {"stop"})
    table.insert(stepList, {"turn", math.rad(90)})
    table.insert(stepList, {"stop"})
    table.insert(stepList, {"repeat"})
    
    -- Target positions for joints
    motorAngleTargetL = 0.0
    motorAngleTargetR = 0.0


    -- To calibrate
    motorAnglePerMetre = 25
    motorAnglePerRadian = 3.1
    
    -- To limit repetitions
    repetitionsLimit = 10
    repetitionsCounter = 0
    
    -- Xlist and YList
    XList = {}
    YList = {}
end

function sysCall_sensing()
    
end


function isCurrentTargetAchieved(posL, posR)

    if (stepList[stepCounter][1] == "stop") then
        -- Check to see if the robot is stationary to within a small threshold
        linearVelocity,angularVelocity=sim.getVelocity(robotBase)
        vLin = math.sqrt(linearVelocity[1]^2 + linearVelocity[2]^2 + linearVelocity[3]^2)
        vAng = math.sqrt(angularVelocity[1]^2 + angularVelocity[2]^2 + angularVelocity[3]^2)
        --print ("stop", linearVelocity, vLin, vAng)
    
        if (vLin < 0.001 and vAng < 0.01) then
            return true
        else
            return false
        end
    end

    -- Start with returnVal = true and negate it if any parts of target are not reached
    returnVal = true
    if (speedBaseL > 0 and posL < motorAngleTargetL) then
        returnVal = false
    end
    if (speedBaseL < 0 and posL > motorAngleTargetL) then
        returnVal = false
    end
    if (speedBaseR > 0 and posR < motorAngleTargetR) then
        returnVal = false
    end
    if (speedBaseR < 0 and posR > motorAngleTargetR) then
        returnVal = false
    end

    return returnVal
end


function sysCall_actuation() 
    if( repetitionsCounter >= repetitionsLimit ) then
        print("XList", XList)
        print("YList", YList)
        xDispesion = dispersionRange(XList)
        print("xDispesion", xDispesion)
        yDispesion = dispersionRange(YList)
        print("yDispersion", yDispesion)
        
        xError = systematicError(XList)
        yError = systematicError(YList)
        print("systematicError", {xError, yError})
        
        covarianza = covarianzaMatrix(XList, YList, {xError, yError})
        
        print("covarianza matriz", covarianza)
        sim.pauseSimulation()
    end


    tt = sim.getSimulationTime()
    -- print("actuation hello", tt)

    -- Get and plot current angles of motor joints
    posL = sim.getJointPosition(leftMotor)
    posR = sim.getJointPosition(rightMotor)
    
    if graphSteps % UPDATE_GRAPHS_EVERY == 0 then
        sim.setGraphUserData(motorGraph,"leftPos",posL)
        sim.setGraphUserData(motorGraph,"rightPos",posR)
        sim.handleGraph(sim.handle_all, tt+sim.getSimulationTimeStep())
    end
    
    graphSteps = graphSteps + 1
 
    -- If we have got to the target of this step: move to the next step
    if (stepCounter == 0 or isCurrentTargetAchieved(posL, posR)) then
        stepCounter = stepCounter + 1
        print("Starting step", stepCounter)
        newStepType = stepList[stepCounter][1]
        newStepAmount = stepList[stepCounter][2]
        print("Step type", newStepType, "Step amount",  newStepAmount)
        if (newStepType == "forward") then
            -- Forward step: set new joint targets
            motorAngleTargetL = posL + newStepAmount * motorAnglePerMetre
            motorAngleTargetR = posR + newStepAmount * motorAnglePerMetre
            speedBaseL = speedBase
            speedBaseR = speedBase
        elseif (newStepType == "turn") then
            -- Turn step: set new targets
            motorAngleTargetL = posL - newStepAmount * motorAnglePerRadian
            motorAngleTargetR = posR + newStepAmount * motorAnglePerRadian
            speedBaseL = -speedBase
            speedBaseR = speedBase
        elseif (newStepType == "stop") then
            speedBaseL = 0
            speedBaseR = 0
        elseif (newStepType == "repeat") then
            -- Drop a `dummy' as a marker of the robot's final position
            newDummy = sim.createDummy(0.05)
            linearPosition = sim.getObjectPosition(robotBase,-1)
            sim.setObjectPosition(newDummy, -1, {linearPosition[1],linearPosition[2],0.0})
            
            repetitionsCounter = repetitionsCounter + 1
            table.insert(XList, linearPosition[1])
            table.insert(YList, linearPosition[2])
            
            -- Teleport robot back to the origin
            -- Just for the purposes of our experiment where we want the robot
            -- to repeat the same motion multiple times  
            resetBase(robotBase, robotStartMatrix)
            -- Regenerate new bumpy floor for next run
            createRandomBumpyFloor()
            stepCounter = 0
            graphSteps = 0
        end
    end


    -- Set the motor velocities for the current step
    sim.setJointTargetVelocity(leftMotor,speedBaseL)
    sim.setJointTargetVelocity(rightMotor,speedBaseR)        
    
    
end

function sysCall_cleanup()
    --simUI.destroy(ui)
end 
