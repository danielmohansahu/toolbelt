import rosbag
import sys
import matplotlib.pyplot as plt
import csv

bag = rosbag.Bag(sys.argv[1])

verbose = False


vx = []
vy = []
vt = []
vs = []

tvx = []
tvy = []
tvt = []

px = []
py = []
pt = []

tx = []
ty = []
tt = []

sx = []
sy = []
st = []

tpsx = []
tpsy = []
tpst = []

tv = []

tWakeUpErrorTime = []
tAfterMoplanTime = []
tSendVelocityTime = []
tCycleEndTime = []
tLoopDurationTime = []
tRunningTime = []

cpu = []
cpuLoad = []

for topic, msg, t in bag.read_messages(topics=['/sewbot/controller/path/metrics']):
    vx.append(msg.velocityOut.linear.x)
    vy.append(msg.velocityOut.linear.y)
    vt.append(msg.velocityOut.angular.z)
    vs.append(msg.sewVelocity)

    tvx.append(msg.targetVelocity.linear.x)
    tvy.append(msg.targetVelocity.linear.y)
    tvt.append(msg.targetVelocity.angular.z)

    px.append(msg.currentPosition.position.x)
    py.append(msg.currentPosition.position.y)
    pt.append(msg.currentPosition.orientation.z)

    tx.append(msg.targetPosition.position.x)
    ty.append(msg.targetPosition.position.y)
    tt.append(msg.targetPosition.orientation.z)

    tpsx.append(msg.targetPositionWithSlip.position.x)
    tpsy.append(msg.targetPositionWithSlip.position.y)
    tpst.append(msg.targetPositionWithSlip.orientation.z)

    sx.append(msg.targetPosition.position.x - msg.targetPositionWithSlip.position.x)
    sy.append(msg.targetPosition.position.y - msg.targetPositionWithSlip.position.y)
    st.append(msg.targetPosition.orientation.z - msg.targetPositionWithSlip.orientation.z)

    tv.append(msg.cycleEndTime)

    tWakeUpErrorTime.append(msg.wakeUpErrorTime)
    tAfterMoplanTime.append(msg.afterMoplanTime)
    tSendVelocityTime.append(msg.sendVelocityTime)
    tCycleEndTime.append(msg.cycleEndTime)
    tLoopDurationTime.append(msg.loopDurationTime)
    tRunningTime.append(msg.runningTime)

    cpu.append(msg.cpu)
    cpuLoad.append(msg.cpuLoad)

bag.close()

# Verbose flag
if len(sys.argv) > 2:
    if '-v' in sys.argv:
        verbose = True

# if len(sys.argv) >= 3:
#     f = open(sys.argv[2], 'wt')
#     try:
#         writer = csv.writer(f)
#         writer.writerow( ('t', 'actualVelocityX', 'actualVelocityY', 'actualVelocityT', 'actualVelocityS', 'targetVelocityX', 'targetVelocityY', 'targetVelocityT', 'targetPositionX', 'targetPositionY', 'targetPositionT', 'currentPositionX', 'currentPositionY', 'currentPositionT') )
#         for i in range(0, len(tv)):
#             writer.writerow( (tv[i], vx[i], vy[i], vt[i], vs[i], tvx[i], tvy[i], tvt[i], tx[i], ty[i], tt[i], px[i], py[i], pt[i]) )
#     finally:
#         f.close()
#
# tU = []
# uU = []
# vU = []
#
# if len(sys.argv) >= 4:
#     f = open(sys.argv[3], 'rb')
#     try:
#         reader = csv.DictReader(f, delimiter=' ')
#         for row in reader:
#             tU.append(float(row['time']))
#             uU.append(100*float(row['updating']))
#             vU.append(1000*float(row['tVel']))
#     finally:
#         f.close()
#
# tv0 = tv[0]
# dt = []
# for i in xrange(0, len(tv)):
#     tv[i] = tv[i] - tv0
#     if i > 0:
#         dt.append(tv[i]-tv[i-1])
#     else:
#         dt.append(0)
#
# for i in xrange(0, len(tU)):
#     tU[i] = tU[i] - tv0

n = 1

###################### POSITION ANALYSIS #######################################

plt.figure(n); n+=1
plt.title('Position Performance')

ax1 = plt.subplot(311)
plt.plot(tv, tx ,'-ob', label = "Target Position")
plt.plot(tv, tpsx ,'-og', label = "Target Position w/ Slip")
plt.plot(tv, px, '-or', label = "Position")
plt.ylabel('X Position (mm)')
plt.grid(True); plt.legend()

ax2 = plt.subplot(312, sharex = ax1)
plt.plot(tv, ty ,'-ob', label = "Target Position")
plt.plot(tv, tpsy ,'-og', label = "Target Position w/ Slip")
plt.plot(tv, py, '-or', label = "Position")
plt.ylabel('Y Position (mm)')
plt.grid(True); plt.legend()

ax3 = plt.subplot(313, sharex = ax1)
plt.plot(tv, [x * 180/3.14159 for x in tt] ,'-ob', label = "Target Position")
plt.plot(tv, [x * 180/3.14159 for x in tpst] ,'-og', label = "Target Position w/ Slip")
plt.plot(tv, [x * 180/3.14159 for x in pt], '-or', label = "Position")
plt.ylabel('Theta Position (deg)')
plt.grid(True); plt.legend()


###################### VELOCITY ANALYSIS #######################################

plt.figure(n); n+=1
plt.title('Velocity Performance')

ax1 = plt.subplot(411)
plt.plot(tv, tvx ,'-ob', label = "Target Velocity")
plt.plot(tv, vx, '-or', label = "Velocity")
plt.ylabel('X Velocity (mm/s)')
plt.grid(True); plt.legend()

ax2 = plt.subplot(412, sharex = ax1)
plt.plot(tv, tvy ,'-ob', label = "Target Velocity")
plt.plot(tv, vy, '-or', label = "Velocity")
plt.ylabel('Y Velocity (mm/s)')
plt.grid(True); plt.legend()

ax3 = plt.subplot(413, sharex = ax1)
plt.plot(tv, [x * 180/3.14159 for x in tvt] ,'-ob', label = "Target Velocity")
plt.plot(tv, [x * 180/3.14159 for x in vt], '-or', label = "Velocity")
plt.ylabel('Theta Velocity (deg/s)')
plt.grid(True); plt.legend()

ax4 = plt.subplot(414, sharex = ax1)
plt.plot(tv, vs, '-or', label = "Velocity")
plt.ylabel('Sew Speed (mm/s)')
plt.grid(True); plt.legend()


######################## TIMING ANALYSIS #######################################

if verbose:
    plt.figure(n); n+=1
    plt.title('Timing Performance')

    ax1 = plt.subplot(211)
    plt.plot(tv, tWakeUpErrorTime, '-or', label='wakeUpErrorTime')
    plt.plot(tv, tAfterMoplanTime, '-ob', label='afterMoplanTime')
    plt.plot(tv, tSendVelocityTime, '-og', label='sendVelocityTime')
    plt.plot(tv, tLoopDurationTime, '-ok', label='loopDurationTime')
    plt.ylabel('Times (s)')
    plt.grid(True); plt.legend()

    ax2 = plt.subplot(212, sharex = ax1)
    plt.plot(tv, tRunningTime, '-oc', label='runningTime')
    plt.ylabel('Time (s)')
    plt.xlabel('Time (s)')
    plt.grid(True); plt.legend()


########################### CPU ANALYSIS #######################################

if verbose:
    plt.figure(n); n+=1
    plt.title('CPU Performance')

    ax1 = plt.subplot(211)
    plt.plot(tv, cpu,'-ob')
    plt.ylabel('CPU (%)')
    plt.grid(True)

    ax2 = plt.subplot(212, sharex = ax1)
    plt.plot(tv, cpuLoad,'-ob')
    plt.ylabel('CPU Load')
    plt.xlabel('time (s)')
    plt.grid(True)

################################################################################

plt.show()
