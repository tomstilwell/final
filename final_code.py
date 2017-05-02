import rospy, math, heapq, numpy, tf, time
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

'''
def getFrontier():
	global gridHeight
	global gridWidth
	global origPosX
	global origPosY
	global startX
	global startY
	global mapRes
	global matrix
	global target
	matrix = [[0 for x in range(gridWidth)] for y in range(gridHeight)] 
	for w in range(gridHeight):
		for z in range(gridWidth):
			matrix[z].append(stuff[w*gridWidth+z])
	mapPositionX = (startX-origPosX)/mapRes
	mapPositiony = (startY-origPosY)/mapRes
	for x in range(int(mapPositionX), (gridWidth-gridWidth%2)/2):
		for y in range((int(mapPositiony)-x), (int(mapPositiony)+x) ):
			if (matrix[x][y]==-1) and ((matrix[x+1][y]<20) or (matrix[x-1][y]<20) or (matrix[x][y+1]<20) or (matrix[x][y-1]<20)):
				target=(x,y)
			if (matrix[int(mapPositionX)-x][y]==-1) and ((matrix[int(mapPositionX)-x+1][y]<20) or (matrix[int(mapPositionX)-x-1][y]<20) or (matrix[int(mapPositionX)-x][y+1]<20) or (matrix[int(mapPositionX)-x][y-1]<20)):
				target=((int(mapPositionX)-x),y)
			if (matrix[int(mapPositionX)+int(mapPositiony)-y][int(mapPositiony)+int(mapPositionX)-x]==-1) and ((matrix[int(mapPositionX)+int(mapPositiony)-y+1][int(mapPositiony)+int(mapPositionX)-x]<20) or (matrix[int(mapPositionX)+int(mapPositiony)-y-1][int(mapPositiony)+int(mapPositionX)-x]<20) or (matrix[int(mapPositionX)+int(mapPositiony)-y][int(mapPositiony)+int(mapPositionX)-x+1]<20) or (matrix[int(mapPositionX)+int(mapPositiony)-y][int(mapPositiony)+int(mapPositionX)-x-1]<20)):
				target=(int(mapPositionX)+int(mapPositiony)-y,int(mapPositiony)+int(mapPositionX)-x)
			if (matrix[int(mapPositionX)+int(mapPositiony)-y][int(mapPositiony)-(int(mapPositionX)-x)]==-1) and ((matrix[int(mapPositionX)+int(mapPositiony)-y+1][int(mapPositiony)-(int(mapPositionX)-x)]<20) or (matrix[int(mapPositionX)+int(mapPositiony)-y-1][int(mapPositiony)-(int(mapPositionX)-x)]<20) or (matrix[int(mapPositionX)+int(mapPositiony)-y][int(mapPositiony)-(int(mapPositionX)-x)+1]<20) or (matrix[int(mapPositionX)+int(mapPositiony)-y][int(mapPositiony)-(int(mapPositionX)-x)-1]<20)):
				target=((int(mapPositionX)+int(mapPositiony)-y), (int(mapPositiony)-(int(mapPositionX)-x)))
			else:
				target =(0,0)
	print("got Frontier")
	return target
'''
def getCloseFrontier():
	global gridHeight
	global gridWidth
	global xPosition
	global yPosition
	global origPosX
	global origPosY
	global mapRes
	global stuff
	print(len(stuff))
	b = gridWidth
	a = gridHeight
	global startX
	global startY
	startX = (xPosition - origPosX)/mapRes
	startY = (yPosition - origPosY)/mapRes
	global closest
	frontiers=[]
	maxDist = math.sqrt(gridWidth**2+gridHeight**2)
	for x in range(gridHeight-3):
		for y in range(gridWidth):
			#print("xcoord: %f ycoord: %f", x, y)
			if ((stuff[x*gridWidth+y]==-1) and ((stuff[(x+1)*gridWidth+y]==0) or (stuff[(x-1)*gridWidth+y]==0) or (stuff[x*gridWidth+y+1]==0) or (stuff[x*gridWidth+y-1]==0))):
				frontiers.append((x,y))
	for element in frontiers:
		dist = math.sqrt((element[0]-startY)**2+(element[1]-startX)**2)
		if (dist<maxDist):
			maxDist = dist
			target = element
			if (stuff[(target[0]+1)*gridWidth+target[1]]==0):
				closest = ((target[0]+1),target[1])
			if (stuff[(target[0]-1)*gridWidth+target[1]]==0):
				closest = ((target[0]-1),target[1])
			if (stuff[(target[0])*gridWidth+target[1]+1]==0):
				closest = ((target[0]),target[1]+1)
			if (stuff[(target[0])*gridWidth+target[1]-1]==0):
				closest = (target[0],target[1]-1)
			else:
				print("there are no more reachable frontiers,")
				print("end of demontration")
	print(closest)

def readBumper(msg):
	if (msg.state == 1):
		print 'BumperEvent'
		#What should happen when the bumper is pressed?
		#executeTrajectory()
		#driveArc(.5,.2,2)
		#spinWheels(.3,.3,1)
		#rotate(math.pi)
		rotate(math.pi)
		#print 'pDrove in an arc'
		#newRotate(math.pi*2,0.5)
		parceData(mapData)
		startX = 0
		startY = 0
		getCloseFrontier()

		lab3cells()
		


def rotate(angle):
	print ("started rotate")
	global odom_list
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10)
	global pose
	global theta
	#Check if angle is within acceptable range
	if(angle > math.pi or angle < -math.pi):
		print "angle is too large or too small"
	else:
		vel = Twist()
		done = False

		#Initial heading
		initialThetaRad = math.radians(theta)

		#Determine which direction to rotate
		if(angle > 0):
			vel.angular.z = 0.5
		else:
			vel.angular.z = -0.5
		while(not done and not rospy.is_shutdown()):
			#Continuously update current heading and difference
			#between initial and current headings
			thetaRad = math.radians(theta)
			diff = thetaRad - initialThetaRad

			#Adjust for values above pi radians and below -pi radians
			if(diff > math.pi):
				error = angle - (diff - 2*math.pi)
			elif(diff < -math.pi):
				error = angle - (diff + 2*math.pi)
			else:
				error = angle - diff

			#Rotate until desired heading is reacched
			if(abs(error) >= math.radians(2.0)):
				pub.publish(vel)
			else:
				done = True
				vel.angular.z = 0
				pub.publish(vel)
	print("finished rotate")
def driveStraight(speed, distance):
	print 'start drive straight'
	global pose
	global pub

	#Initial x and y positions of the turtlebot
	initialX = xPosition
	initialY = yPosition

	#Create two Twist messages
	drive_msg = Twist()
	stop_msg = Twist()

	#Populate messages with data
	drive_msg.linear.x = speed
	stop_msg.linear.x = 0
	atTarget = False
	print 'about to while'
	while(not atTarget and not rospy.is_shutdown()):
		
		#Continously find the distance travelled from starting position
		currentX = xPosition
		currentY = yPosition
		currentDistance = math.sqrt(math.pow((currentX - initialX), 2) + math.pow((currentY - initialY), 2))
		#Drive until the robot has reached its desired positon
		if(currentDistance >= distance):
			atTarget = True
			pub.publish(stop_msg)
		else:
			pub.publish(drive_msg)
			rospy.sleep(0.15)
def navToPose(goal):
	global pose
	print("navToPose")
	goalPoseX = goal.pose.position.x	#x position of the goal
	goalPoseY = goal.pose.position.y	#y position of the goal
	odomW = goal.pose.orientation
	q = [odomW.x, odomW.y, odomW.z, odomW.w]
	roll, pitch, yaw = euler_from_quaternion(q)
	goalPoseAng = yaw					#orientation of goal
	initialX = xPosition				#Starting x position of turtlebot
	initialY = yPosition
	initialAng = math.radians(theta)				#Starting y position of turtlebot
	#Rotate towards goal
	if((goalPoseX - initialX) == 0):
		if((goalPoseY - initialY) > 0):
			print "spin!"
			rotate((math.pi/2.0) - initialAng)
		elif((goalPoseY - initialY) < 0):
			print "spin!"
			rotate(-(math.pi/2.0) - initialAng)
	else:
		print "spin!"
		rotate(math.atan2((goalPoseY - initialY), (goalPoseX - initialX)) - initialAng)
	#Drive towards goal
	print "move!"
	driveStraight(0.2, math.sqrt(math.pow((goalPoseX - initialX), 2) + math.pow((goalPoseY - initialY), 2)))
	initialAng = math.radians(theta)	#Heading of turtlebot after reaching desired location
	#Rotate to pose
	if((goalPoseAng - initialAng) != 0):
		if((goalPoseAng - initialAng) > math.pi):
			print "spin!"
			rotate((goalPoseAng - initialAng) - 2*math.pi)
		elif((goalPoseAng - initialAng) < -math.pi):
			print "spin!"
			rotate((goalPoseAng - initialAng) + 2*math.pi)
		else:
			print "spin!"
			rotate(goalPoseAng - initialAng)
	print "done"
	rotate(math.pi)
	#print 'pDrove in an arc'
	#newRotate(math.pi*2,0.5)
	parceData(mapData)
	startX = 0
	startY = 0
	getCloseFrontier()

	lab3cells()

def getPosition(data):
	print("new start point")
	global mapData
	parceData(mapData)
	global startX
	global startY	
	global gridHeight
	global gridWidth
	global origPosX
	global origPosY
	global mapRes
	global stuff
	#startX = int((data.pose.pose.position.x+origPosX)/mapRes)
	#startY = int((data.pose.pose.position.y+origPosY)/mapRes)
	getCloseFrontier()
	lab3cells()
	'''
	s = AStar()
	s.init_grid()
	print("init grid")	
	s.process()
	print("finished process")
	s.get_path()
	'''
	

def callbackMap(data):
	print('called map callback')
	global mapData
	mapData = OccupancyGrid()
	mapData = data

def parceData(OccupancyGrid):
	print('parcing map data')
	global mapData
	global gridHeight
	global gridWidth
	global origPosX
	global origPosY
	global mapRes
	global stuff
	data = mapData
	mapRes = data.info.resolution
	origPosX = data.info.origin.position.x
	origPosY = data.info.origin.position.y
	gridHeight = data.info.height
	gridWidth = data.info.width
	stuff = data.data
	print('parced')
	
def printTarget():
	print ('printing')
	pub = rospy.Publisher('colorcells2', GridCells, queue_size=1)
	global mapRes
	global startX
	global startY
	global gridWidth
	global gridHeight
	global stuff
	global origPosX
	global origPosY
	global closest
	print(closest)
	grid = GridCells()
	grid.header.frame_id = 'map'
	grid.cell_width = mapRes
	grid.cell_height = mapRes
	global xFinal
	global yFinal
	xFinal = (closest[0])*mapRes+origPosX
	yFinal = (closest[1])*mapRes+origPosY
	t=Point()
	t.y = xFinal
	t.x = yFinal
	t.z = 0

	grid.cells.append(t)
	while 1:
		pub.publish(grid)
		
def lab3cells():
	print ('printing')
	pub1 = rospy.Publisher('colorcells', GridCells, queue_size=1)
	pub2 = rospy.Publisher('colorcells2', GridCells, queue_size=1)
	global mapRes
	global gridWidth
	global gridHeight
	global stuff
	global origPosX
	global origPosY
	grid = GridCells()
	grid.header.frame_id = 'map'
	grid.cell_width = mapRes
	grid.cell_height = mapRes
	a = gridWidth
	b = gridHeight
	'''
	for x in range(gridHeight-3):
		for y in range(gridWidth):
			if ((stuff[x*gridWidth+y]==-1) and ((stuff[(x+1)*gridWidth+y]<20) and (stuff[(x+1)*gridWidth+y]>=0))or ((stuff[(x-1)*gridWidth+y]>=0) and (stuff[(x-1)*gridWidth+y]<20)) or ((stuff[x*gridWidth+y+1]>=0)and (stuff[x*gridWidth+y+1]<20)) or ((stuff[x*gridWidth+y-1]>=0) and stuff[x*gridWidth+y-1]<20)):
				t=Point()
				t.y = x*mapRes+origPosY
				
				t.x = y*mapRes+origPosX
				t.z = 0
				grid.cells.append(t)
	'''
	for x in range(gridHeight-3):
		for y in range(gridWidth):
			if ((stuff[x*gridWidth+y]==-1) and ((stuff[(x+1)*gridWidth+y]==0) or (stuff[(x-1)*gridWidth+y]==0) or (stuff[x*gridWidth+y+1]==0) or (stuff[x*gridWidth+y-1]==0))):
				t=Point()
				t.y = x*mapRes+origPosY
				
				t.x = y*mapRes+origPosX
				t.z = 0
				grid.cells.append(t)
	
	global xFinal
	global startX
	global startY
	global yFinal
	global closest
	grid2 = GridCells()
	grid2.header.frame_id = 'map'
	grid2.cell_width = mapRes*5
	grid2.cell_height = mapRes*5
	xFinal = (closest[0])*mapRes+origPosY
	yFinal = (closest[1])*mapRes+origPosX
	u=Point()
	u.y = xFinal
	u.x = yFinal
	u.z = 0
	grid2.cells.append(u)
	print("goal is")
	print(closest)
	print("ptint data")
	print(mapRes)
	print(origPosX)
	print(origPosY)
	print(grid.cells[5])
	print(grid2.cells[0])	
	#driveGoal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
	global closest
	#a = targetToPoseStamped(closest)
	#driveGoal.publish(a)
	#navToPose(a)
	print('publishing')
	tEnd = time.time()+5
	while time.time()<tEnd:
		pub1.publish(grid)
		pub2.publish(grid2)

def callbackstart(data):
	print("i used callback start")	
	rotate(math.pi)
	rotate(math.pi)
	#print 'pDrove in an arc'
	#newRotate(math.pi*2,0.5)
	parceData(mapData)
	startX = 0
	startY = 0
	getCloseFrontier()

	lab3cells()	
def callbackCostMap(data):
	print('called costmap callback')
	global mapData
	mapData = OccupancyGrid()
	mapData = data
	print (data.info.height)
	print (data.info.width)
	#parceData(data)
	#lab3cells()
def targetToPoseStamped(target):
	global mapRes
	global origPosX
	global origPosY
	a = target[0]*mapRes+origPosX
	b = target[1]*mapRes+origPosY

	goal = PoseStamped()
	goal.header.frame_id = 'map'
	goal.pose.position.x = a
	goal.pose.position.y = b
	goal.pose.position.z = 0
	goal.pose.orientation.x = 0
	goal.pose.orientation.y = 0
	goal.pose.orientation.z = 0
 	goal.pose.orientation.w = 1

 	return goal
def drive():
	driveGoal = rospy.Publisher('move_base_simple/goal', PoseStamped, navToPose, queue_size = 1)
	global closest
	a = targetToPoseStamped(closest)
	while 1:
		driveGoal.pub(a)
class Cell(object):
	def __init__(self, x, y, reachable):
		self.reachable = reachable
		self.x = x
		self.y = y
		self.parent = None
		self.g = 0
		self.h = 0
		self.f = 0
class AStar(object):
	def __init__(self):
			self.opened = []
			heapq.heapify(self.opened)
			self.closed = set()
			self.cells = []
			global gridHeight
			global gridWidth
			self.grid_height = gridHeight
			self.grid_width = gridWidth
#creates a grid defining cells by their coordinates and whether or not they are reachable
	def init_grid(self):
		global stuff

		for x in range(gridWidth):
			for y in range(gridHeight):
				if (stuff[gridWidth*x+y]==100 or stuff[gridWidth*x+y]==-1):
					reachable = False
				else:
					reachable = True
				self.cells.append(Cell(x, y, reachable))
		
		global closest		 
		endX = closest[0]
		endY = closest[1]

		global startX
		global startY
		self.start = self.get_cell(int(startX), int(startY))
		self.end = self.get_cell(187, 327)
		print (self.start.x)
		print (self.start.y)
		print(self.end.reachable)
		print(self.end.x)
		print(self.end.y)
	def get_heuristic(self, cell):
		return 10 * (abs(cell.x - self.end.x)+abs(cell.y- self.end.y))

	def get_cell(self, x, y):
		#pdb.set_trace()
		return self.cells[x * self.grid_height + y]

	def get_adjacent_cells(self, cell):
			cells = []
			if cell.x < self.grid_width-1:
				cells.append(self.get_cell(cell.x+1, cell.y))
			if cell.y > 0:
				cells.append(self.get_cell(cell.x, cell.y-1))
			if cell.x > 0:
				cells.append(self.get_cell(cell.x-1, cell.y))
			if cell.y < self.grid_height-1:
				cells.append(self.get_cell(cell.x, cell.y+1))
			return cells
#follows the list of parent cells to return the correct path to take
	def get_path(self):
		cell = self.end
		global path
		path = [(cell.x, cell.y)]
		while cell.parent is not self.start:
			cell = cell.parent
			path.append((cell.x, cell.y))
			print('added cell to path')
		path.append((self.start.x, self.start.y))
		path.reverse()
		#print(path)
		return path

	def display_path(self):
		cell = self.end
		print(cell)
		while cell.parent is not self.start:
			cell = cell.parent
			print 'path: cell: %d,%d' % (cell.x, cell.y)
			
	def update_cell(self, adj, cell):
		adj.g = cell.g + 10
		adj.h = self.get_heuristic(adj)
		adj.parent = cell
		adj.f = adj.h + adj.g
#runs A*, if the grid is initialized, adds the correct order of parent cells to self
	def process(self):
		heapq.heappush(self.opened, (self.start.f, self.start))
		while len(self.opened):
			f, cell = heapq.heappop(self.opened)
			self.closed.add(cell)
			if cell is (self.end):
				self.display_path()
				break
		adj_cells = self.get_adjacent_cells(cell)
		for adj_cell in adj_cells:
			if adj_cell.reachable and adj_cell not in self.closed:
				if (adj_cell.f, adj_cell) in self.opened:
					if adj_cell.g > cell.g + 10:
						print(cell.x)
						self.update_cell(adj_cell, cell)
				else:
					self.update_cell(adj_cell, cell)
					heapq.heappush(self.opened, (adj_cell.f, adj_cell))
		print("end parent")
		print(self.end.parent)





def timerCallback(event):
	global pose
	global xPosition
	global yPosition
	global theta

	pose = Pose()

	odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(10))
	(position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
	pose.position.x = position[0]
	pose.position.y = position[1]
	xPosition = position[0]
	yPosition = position[1]

	odomW = orientation
	q = [odomW[0], odomW[1], odomW[2], odomW[3]]
	roll, pitch, yaw = euler_from_quaternion(q)
	pose.orientation.z = yaw
	theta = math.degrees(yaw)
	

def main():
	global pub
	global pose
	global odom_tf
	global odom_list
	pose = Pose()
	rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, callbackstart)
	rospy.init_node('frontierfinder', anonymous = True)
	rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callbackCostMap)
	rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, getPosition)
	rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) 
	#rospy.Subscriber('move_base_simple/goal', PoseStamped, navToPose, queue_size=1)
	odom_list = tf.TransformListener()
	timerCallback(1)
	#
	#
	rospy.Timer(rospy.Duration(0.01), timerCallback)
	odom_list = tf.TransformListener()
	print("timer callback, you may press the bumper...")
	
	#rospy.Subscriber("/map", OccupancyGrid, callbackMap)
	rospy.spin()
if __name__ == '__main__':
	main()
