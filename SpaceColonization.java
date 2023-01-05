//PSEUDOCODE

class Agent {
	
	int position;
	int goal;
	int[] markers;
	int[] weights;
	int radius;
	int speed;
	boolean find = false;
	
	
	
	public static int funkcija (x, y) {
	  return (1/ (1 + length(y))* (1+ (x.dot(y)) / length(x)*length(y)))
  }
	
	 public static int[] weights() {
		this.weights = [];
		for (int i = 0; i < this.markers.length; i++) {
			int suma = 0;
			for (int j = 0; j < this.markers.length; j++) {
				suma = suma + funkcija(goal-position, this.markers[j]);
			}
			int weight = funkcija(goal-position, this.markers[i]) / suma;
			this.weights[i] = weight;
		}
    this.weights;
  }
  
  
  
  public static int motionVector() {
	  int m = 0;
	  for (int i=0; i<this.markers.length; i++) {
		  m = m + this.weights[i]*(markers[i] - this.position);
	  }
	  return m;
  }
  
	
	public static int newPosition() {
		return speed*motionVector()/motionVector().Length;
	}
	
	public static boolean success() {
		if (newPosition() < this.radius) {
			return true;
		}
		return false;
	}
  }
  
 class Model {
	
	int [] markers;
    Agent [] agents;
	int sizeArea;
	[] area;
	
	//add markers on area
	for(int i = 0; i < this.markers.length; i++) {
		int x = Math.random(1, sizeArea);
		int y = Math.random(1, sizeArea);
		int z = Math.random(1, sizeArea);
		area.add(new Marker(x, y, z));
	}
	
	 // add agents, put start position and goal
    for (let i = 0; i < this.agents.length; i++) {
      area.add(new Agent(Math.random(1, sizeArea), Math.random(1, sizeArea)));
    }
  }
 