import matplotlib.pyplot as plt
import numpy as np



t=np.genfromtxt("D:/Acceleration.csv",dtype=float,delimiter=',',skip_header=2,usecols=(0,),skip_footer=1)
x=np.genfromtxt("D:/Acceleration.csv",dtype=float,delimiter=',',skip_header=2,usecols=(1,),skip_footer=1)
y=np.genfromtxt("D:/Acceleration.csv",dtype=float,delimiter=',',skip_header=2,usecols=(2,),skip_footer=1)
z=np.genfromtxt("D:/Acceleration.csv",dtype=float,delimiter=',',skip_header=2,usecols=(3,),skip_footer=1)

fig=plt.figure(figsize=(18,12))
fig.suptitle("ACCELERATION DATA",size=32)

x_vs_t=fig.add_subplot(1,2,1)
y_vs_t=fig.add_subplot(2,2,2)
z_vs_t=fig.add_subplot(2,2,4)

x_vs_t.plot(t,x,color='red')
y_vs_t.plot(t,y,color='green')
z_vs_t.plot(t,z,color='blue')

x_vs_t.set_xlabel("Time (s)")
y_vs_t.set_xlabel("Time (s)")
z_vs_t.set_xlabel("Time (s)")

x_vs_t.set_ylabel("Acceleration (g's)")
y_vs_t.set_ylabel("Acceleration (g's)")
z_vs_t.set_ylabel("Acceleration (g's)")

x_vs_t.set_xlim(0,max(t))
y_vs_t.set_xlim(0,max(t))
z_vs_t.set_xlim(0,max(t))

x_vs_t.set_ylim(-8,8)
y_vs_t.set_ylim(-8,8)
z_vs_t.set_ylim(-8,8)

x_vs_t.set_title("X axis")
y_vs_t.set_title("Y axis")
z_vs_t.set_title("Z axis")


plt.show()
