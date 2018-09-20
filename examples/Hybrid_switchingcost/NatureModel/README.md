The following changes are made in backup2:

* m = 1 now (as opposed to 0.2 in the previous version)
* desired step length is d_des = 0.7 now (as opposed to 0.6 in the previous version)

The following changes are made:

* Trajectories now start from mode 1, with l = locally minimum l, ldot = 0, theta = 0, thetadot > 0
* To make sure mode 2 lasts for long enough time, the guards are defined as:
	1->2:	y >= yRhigh
	2->1: 	y <= yRlow

Did not work well for us. Worked fine for gpops. Need to figure out why.

