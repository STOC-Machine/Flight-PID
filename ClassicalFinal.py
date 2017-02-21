from __future__ import division
from visual import *
from visual.graph import *

scene.autocenter=True
scene.width=800
scene.height=800

#This frame is the whole quadcopter
qcopter=frame(make_trail=True)

#These frames are the four rotors
r1=frame(frame=qcopter,pos=(1,.06,0))
r2=frame(frame=qcopter,pos=(-1,.06,0))
r3=frame(frame=qcopter,pos=(0,.06,1))
r4=frame(frame=qcopter,pos=(0,.06,-1))

#The x-shaped body of the quadcopter items
xbody = box(frame=qcopter, pos=(0,0,0), size=(2,.1,.1), color=color.green)
ybody = box(frame=qcopter, pos=(0,0,0), size=(.1,.1,2), color=color.blue)

#Here are the items that form the rotors
#rotor1
p0=box(frame=r1, size=(.5,.01,.05),color=color.red)
p1=box(frame=r1, size=(.05,.01,.5),color=color.red)
#rotor2
p2=box(frame=r2, size=(.5,.01,.05),color=color.red)
p3=box(frame=r2, size=(.05,.01,.5),color=color.red)
#rotor3
p4=box(frame=r3, size=(.5,.01,.05),color=color.red)
p5=box(frame=r3, size=(.05,.01,.5),color=color.red)
#rotor4
p6=box(frame=r4, size=(.5,.01,.05),color=color.red)
p7=box(frame=r4, size=(.05,.01,.5),color=color.red)

qcopter.pos.y=1

################# Variable Initialization Section #########################

'''These 12 variables and their time derivatives make up the 6 degrees of freedom'''
pn=0        #inertial north-south position along i in Frame i
pndot=0
pe=0        #inertial east-west position along j in Frame i
pedot=0
h=1         #altitude along k in Frame i
hdot=0
u=0         #body frame velocity measured along i in Frame b
udot=0
v=0         #body frame velocity measured along j in Frame b
vdot=0
w=0         #body frame velocity measured along k in Frame b
wdot=0
phi=0       #roll angle wrt Frame v2
phidot=0
theta=0     #pitch angle wrt Frame v1
thetadot=0
psi=0       #yaw angle wrt Frame v
psidot=0
p=0         #roll rate measured along i in Frame b
pdot=0
q=0         #pitch rate measured along i in Frame b
qdot=0
r=0         #yaw rate measured along i in Frame b
rdot=0

'''These variables are the phisical constants of the system
   System is approximated as a central spherical mass, w/ 4 point masses'''
M=5        #mass of central spherical mass
m=1         #mass of each pont mass
L=2         #distance from motor to sphere
R=.25        #radius of shpere
k1=1        #motor force constant
k2=.5       #motor torque constant
g=2         #gravitational constant

'''Moments of Inertia'''
Jx=(2*M*R*R)/5+2*L*L*m
Jy=(2*M*R*R)/5+2*L*L*m
Jz=(2*M*R*R)/5+4*L*L*m

'''These are the motor control variables'''
mcf=1      #motor controls for motors f, r, b, l
mcr=2
mcb=.1
mcl=1


'''Forces and Torques'''
Ff=k1*mcf  #Forces of motors f, r, b, l
Fr=k1*mcr
Fb=k1*mcb
Fl=k1*mcl

Tf=k2*mcf  #Horizontal troques of motors f, r, b, l
Tr=k2*mcr
Tb=k2*mcb
Tl=k2*mcl

F=Ff+Fr+Fb+Fl     #Net force 

Tphi=L*(Fl-Fr)    #Torque in the roll direction
Ttheta=L*(Ff-Fb)  #Torque in the pitch direction
Tpsi=Tr+Tl-Tf-Tb  #Torque in the yaw direction


'''Time Variables'''
t=0
dt=.001


'''PID related variables'''
cphi=0
kpphi=10
kiphi=0
kdphi=0
Iphi=0

ctheta=0
kptheta=10
kitheta=0
kdtheta=0
Itheta=0

kppsi=10
kipsi=0
kdpsi=0
Ipsi=0

############################## Setup the graphs #############################
gd = gdisplay(title='Pitch', y = 0, x = 785, height = 200, width=400) 
hd = gdisplay(title='Roll', y = 200, x = 785, height = 200, width=400)
jd = gdisplay(title='Yaw', y = 400, x = 785, height = 200, width=400)

xd = gdisplay(title='X Position', y = 000, x = 1170, height = 200, width=400)
yd = gdisplay(title='Y Position', y = 200, x = 1170, height = 200, width=400)
zd = gdisplay(title='Z Position', y = 400, x = 1170, height = 200, width=400)

qcopter_roll = gcurve(gdisplay=hd, color=color.green)
qcopter_roll.plot(pos=(t,phi))

qcopter_pitch = gcurve(gdisplay=gd, color=color.red) 
qcopter_pitch.plot(pos=(t,theta))

qcopter_yaw = gcurve(gdisplay=jd, color=color.yellow)
qcopter_yaw.plot(pos=(t,psi))

qcopter_x = gcurve(gdisplay=xd, color=color.red)
qcopter_x.plot(pos=(t,qcopter.pos.x))

qcopter_y = gcurve(gdisplay=yd, color=color.green)
qcopter_y.plot(pos=(t,qcopter.pos.y))

qcopter_z = gcurve(gdisplay=zd, color=color.yellow)
qcopter_z.plot(pos=(t,qcopter.pos.z))


i=0
################################ Main Loop ##############################
while True:
    rate(1000)

    #Here the time derivatives of the 12 core variables are calculated
    pndot=cos(theta)*cos(psi)*u+(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v+(cos(phi)*sin(theta)*cos(psi)+sin(theta)*sin(psi))*w
    pedot=cos(theta)*sin(psi)*u+(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v+(cos(phi)*sin(theta)*sin(psi)-sin(theta)*cos(psi))*w
    hdot=sin(theta)*u-sin(phi)*cos(theta)*v-cos(phi)*cos(theta)*w
    udot=r*v-q*w-g*sin(theta)
    vdot=p*w-r*u+g*cos(theta)*sin(phi)
    wdot=q*u-p*v+g*cos(theta)*cos(phi)-(F/m)
    phidot=p+sin(phi)*tan(theta)*q+cos(phi)*tan(theta)*r
    thetadot=cos(phi)*q-sin(phi)*r
    psidot=(sin(phi)/cos(theta))*q+(cos(phi)/cos(theta))*r
    pdot=((Jy-Jz)/Jx)*q*r+(Tphi/Jx)
    qdot=((Jz-Jx)/Jy)*p*r+(Ttheta/Jy)
    rdot=((Jx-Jy)/Jz)*q*p+(Tpsi/Jz)

    #Here the values of the 12 core variables are updated
    pn=pn+pndot*dt 
    pe=pe+pedot*dt
    h=h+hdot*dt
    u=u+udot*dt
    v=v+vdot*dt
    w=w+wdot*dt
    phi=phi+phidot*dt
    theta=theta+thetadot*dt
    psi=psi+psidot*dt
    p=p+pdot*dt
    q=q+qdot*dt
    r=r+rdot*dt

    '''
    if i%5==0:
        #Here the PID control is executed
        Iphi+=(cphi-phi)*dt
        Tphi=L*(Fl-Fr)+kpphi*(cphi-phi)-kdphi*(cphi-phi)+kiphi*Iphi

        Ttheta+=(ctheta-theta)*dt
        Ttheta=L*(Ff-Fb)+kptheta*(ctheta-theta)-kdtheta*(ctheta-theta)+kitheta*Itheta
        
        Ipsi+=psi*dt
        Tpsi=Tr+Tl-Tf-Tb-kppsi*psi-kdpsi*r-kipsi*Ipsi
    '''
    #Now the visual is updated to reflect the changes
    qcopter.x=pn
    qcopter.z=pe
    qcopter.y=h

    qcopter.axis=(cos(theta)*cos(psi),-cos(theta)*sin(psi),sin(theta))
    qcopter.up=(sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi),cos(phi)*cos(theta),sin(phi)*cos(psi)-cos(phi)*sin(theta)*sin(psi))

    #update various graphs
    qcopter_pitch.plot(pos=(t,theta))
    qcopter_roll.plot(pos=(t,phi))
    qcopter_yaw.plot(pos=(t,psi))
    qcopter_x.plot(pos=(t,qcopter.pos.x))
    qcopter_y.plot(pos=(t,qcopter.pos.y))
    qcopter_z.plot(pos=(t,qcopter.pos.z))

    if qcopter.pos.y < 0:
        print t
        break
    
    t=t+dt
    i+=1
