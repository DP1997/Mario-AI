package mirodopf_Agent;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import ch.idsia.mario.engine.LevelScene;
import ch.idsia.mario.engine.sprites.Mario.STATUS;
import ch.idsia.mario.environments.Environment;
import de.novatec.mario.engine.generalization.Coordinates;
import de.novatec.mario.engine.generalization.Entities.EntityType;
import de.novatec.mario.engine.generalization.Entity;
import de.novatec.mario.engine.generalization.Tile;
import de.novatec.mario.engine.generalization.Tiles.TileType;
import de.novatec.marioai.tools.LevelConfig;
import de.novatec.marioai.tools.MarioAiRunner;
import de.novatec.marioai.tools.MarioInput;
import de.novatec.marioai.tools.MarioKey;
import de.novatec.marioai.tools.MarioNtAgent;

enum State {
	COIN, ENEMY, GOAL
}
public class MarioAgent extends MarioNtAgent {
	private static final int TARGET_OFFSET=Environment.HalfObsWidth*10;
	private static State state;
	private static List<MarioInput> inputs = new ArrayList<>();
	private static List<Tile> collectables;
	private static List<Entity> enemies;
	private static Node goal;
	private long startTime, currentTime;
	
	@Override
	public String getName() {
		return "mirodopf";
	}

	@Override
	public MarioInput doAiLogic() {
		startTime = System.currentTimeMillis();
		LevelScene scene = getAStarCopyOfLevelScene();
		goal = new Node(scene, new MarioInput());
		goal.xPos = scene.getLevelXExit()*16; //Map-Koordinate
		goal.yPos = 0;
		state = State.GOAL;
		
		collectables = getInteractiveBlocksOnScreen();
		for (Tile t : collectables) {
			if (t.getType() == TileType.COIN || t.getType() == TileType.FLOWER_POT) {
				goal.xPos =  t.getCoords().getX();
				goal.yPos =  t.getCoords().getY();
				state = State.COIN;
				break;
			}
		}
		/*
		enemies = getAllEnemiesOnScreen();
		for (Entity e : enemies) {
			if (e.isSquishable()) {
				goal.xPos = e.getCoordinates().getX();
				goal.yPos = e.getCoordinates().getY();
				return getPath(scene, goal);
			}
		}
		*/
		return getPath(scene, goal);
	}
	
	private MarioInput getPath(LevelScene scene, Node goal) {
		Node start=new Node(scene,null);
		List<Node> openSet;
		List<Node> closedSet;
		Map<Node,Double> gMap;
		Map<Node,Double> fMap;
		
			openSet=new ArrayList<>();
			closedSet=new ArrayList<>();
			gMap=new HashMap<>();
			fMap=new HashMap<>();
		
		openSet.add(start);
		gMap.put(start, 0.0); //cost for start is 0
		fMap.put(start, getDistanceFromTo(start.getX(), start.getY(), goal.xPos, goal.yPos)); //heuristic estimate
		
		Node actual=null;
		List<Node> nextNodes=new ArrayList<>();

		while(!openSet.isEmpty()) { //while open set is not empty or planing target reached
			currentTime = System.currentTimeMillis();
			//pick best node from open list with lowest cost
			actual=openSet.get(0);
			for(Node next: openSet) {
				if(fMap.get(next)<fMap.get(actual)) {
					actual=next;
				}
			}
			
			if(state==State.COIN) {
				//reached coin
				if(Math.abs(actual.getX()-goal.xPos)<1 && Math.abs(actual.getY()-goal.yPos)<1|| currentTime-startTime>=200) {
					break;
				}
			}
			else if (state==State.GOAL) {
				//reached right edge of view, win or timelimit exceeded
				if((actual.getX()>=scene.getMarioX()+TARGET_OFFSET)
						|| actual.getLevelScene().getMarioStatus()==STATUS.WIN
						|| currentTime-startTime>=200) {
						break; 
					}
			}
				
			nextNodes.clear();
			nextNodes=getAllNodes(actual, inputs); //generate next node list;
			
			openSet.remove(actual);
			closedSet.add(actual);

			//for each neighbor
				// if not in closed list ->continue
				// if not in open list ->add to open list
			for(Node neighbor: nextNodes) {
				if(closedSet.contains(neighbor)) continue;
				
				if(!openSet.contains(neighbor)) openSet.add(neighbor);
				
				//get costs till now
				double tmpGScore=gMap.get(actual)+actual.getDistanceTo(neighbor);
				
				//check scores -> if path from actual to neighbor better than old way replace it
				if(gMap.get(neighbor)!=null&&tmpGScore>gMap.get(neighbor)) continue; //check if old path is better
				
				//else set this way
				neighbor.setParent(actual);
				gMap.put(neighbor, tmpGScore);
				fMap.put(neighbor, tmpGScore+getEvaluation(neighbor, actual)+getDistanceFromTo(neighbor.getX(), neighbor.getY(), goal.xPos, goal.yPos));
			} //for each nextNodes
		} //while
		List<Node> tmp=reconstructPath(actual);
		if(tmp.size()>=2)return tmp.get(tmp.size()-2).getMarioInput();
		return getMarioInput();
	}
	
	protected List<Node> reconstructPath(Node last){
		List<Node> path=new ArrayList<>();
		Node parent=last;
		
		while(parent!=null) {
			path.add(parent);
			addCoordToDraw(new Coordinates(parent.getX(),parent.getY()));
			parent=parent.getParent();
		}
		return path;
	}
	
	private LevelScene tick(LevelScene actual, MarioInput input) {
		LevelScene scene = actual.getAStarCopy();

		scene.setMarioInput(input);
		scene.tick();
		scene.tick();
		
		return scene;
	}
	
	private List<Node> getAllNodes(Node parent,List<MarioInput> inputs){
		List<Node> nodes=new ArrayList<>();
		LevelScene actual=parent.getLevelScene();
		
		for(MarioInput next: inputs) {
			LevelScene clone=tick(actual,next);
			if (clone.getMarioStatus() == STATUS.LOSE || clone.getTimesMarioHurt() > actual.getTimesMarioHurt()) continue;
			nodes.add(new Node(clone,next));
			//addCoordToDraw(new Coordinates(clone.getMarioX(),clone.getMarioY()));
		}
	
		return nodes;
	}
	
	protected static double getDistanceFromTo(float x1,float y1,float x2, float y2) { // VERY simple heuristic, will get you to the target
		return Math.sqrt(Math.pow(x2-x1, 2)+Math.pow(y2-y1, 2));
	}
	
	private float getEvaluation(Node succ, Node parent) {
		float evaluation =
					- succ.getX()*2
					- succ.getLevelScene().getMarioXA()*12
				   	- succ.usedScene.getMarioCoins()*16
					- succ.usedScene.getKilledCreaturesByStomp()*54
					- succ.usedScene.getKilledCreaturesByShell()*59
					- succ.usedScene.getKilledCreaturesByFireBall()*46
					- succ.usedScene.getMarioGainedFowers()*58
					- succ.usedScene.getMarioGainedMushrooms()*58;
					return evaluation;
	}
	
	public static void main (String [] args) {
		MarioInput input;
		
//		input=new MarioInput();
//		input.press(MarioKey.LEFT);
//		inputs.add(input);
		
		input=new MarioInput();
		input.press(MarioKey.LEFT);
		input.press(MarioKey.SPEED);
		inputs.add(input);
		
//		input=new MarioInput();
//		input.press(MarioKey.LEFT);
//		input.press(MarioKey.JUMP);
//		inputs.add(input);
//		
//		input=new MarioInput();
//		input.press(MarioKey.LEFT);
//		input.press(MarioKey.SPEED);
//		input.press(MarioKey.JUMP);
//		inputs.add(input);
		
		input=new MarioInput();
		input.press(MarioKey.RIGHT);
		input.press(MarioKey.SPEED);
		input.press(MarioKey.JUMP);
		inputs.add(input);
		
		input=new MarioInput();
		input.press(MarioKey.RIGHT);
		input.press(MarioKey.SPEED);
		inputs.add(input);
		
//		input=new MarioInput();
//		input.press(MarioKey.RIGHT);
//		inputs.add(input);
		
		input=new MarioInput();
		input.press(MarioKey.JUMP);
		inputs.add(input);
		
//		input=new MarioInput();
//		input.press(MarioKey.DOWN);
//		inputs.add(input);
		
//		input=new MarioInput();
//		inputs.add(null);
		
		MarioAiRunner.run(new MarioAgent(), LevelConfig.LEVEL_6, 24, 2, true, true, true);
	}
	
}