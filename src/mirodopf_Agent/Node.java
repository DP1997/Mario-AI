package mirodopf_Agent;


import ch.idsia.mario.engine.LevelScene;
import de.novatec.marioai.tools.MarioInput;

public class Node {
	private static final double EPSILON=2;

	public LevelScene usedScene;
	public MarioInput input;
	public Node parent;
	public float xPos;
	public float yPos;
	public float velocity = 0;

	public Node(LevelScene usedScene, MarioInput input) {
		this.usedScene=usedScene;
		this.input=input;
	}

	public float getX() {
		return usedScene.getMarioX();
	}

	public float getY() {
		return usedScene.getMarioY();
	}

	public LevelScene getLevelScene() {
		return this.usedScene;
	}

	public Node getParent() {
		return this.parent;
	}

	public void setParent(Node parent) {
		this.parent=parent;
	}

	public MarioInput getMarioInput() {
		return this.input;
	}

	public double getDistanceTo(Node other) {
		return Math.sqrt(Math.pow(other.getX()-this.getX(), 2)+Math.pow(other.getY()-this.getY(), 2));
	}

	public String toString() {
		return "Node: x="+getX()+", y="+getY();
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((input == null) ? 0 : input.hashCode());
		result = prime * result + Float.floatToIntBits(velocity);
		result = prime * result + Float.floatToIntBits(xPos);
		result = prime * result + Float.floatToIntBits(yPos);
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Node other = (Node) obj;
		if (usedScene == null) {
			if (other.usedScene != null)
				return false;
		} else {
			if(Math.abs(this.getX()-other.getX())<EPSILON&&Math.abs(this.getY()-other.getY())<EPSILON) return true;

		else return false;}
		return true;
	}



}
