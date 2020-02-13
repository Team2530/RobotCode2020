package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class Pixy extends SubsystemBase {
  private final Pixy2 pixy;

  // ints for ball data
  Block loadedBlock;
  int ballWidth;
  int ballHeight;
  int ballX;
  int ballY;
  int ballAngle;
  int ballAge;
  int lastBlockCount;

  public Pixy() {
    // Initializes the Pixy
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init();
    loadedBlock = this.getBiggestBlock();
    pixy.setLamp((byte) 1, (byte) 1);
    lastBlockCount = 0;
  }

  public Block getBiggestBlock() {
    // Finds blocks that match signature 1,
    // finds the largest of them,
    // then returns it in the original data-retrievable format
    
    int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG2, 25 /* limit on blocks to return */);
    if(blockCount < 0){
      blockCount = lastBlockCount;
    }
    lastBlockCount = blockCount;
    SmartDashboard.putNumber("block count",  blockCount);

    if (blockCount <= 0) {
      return null; // If blocks were not found, stop processing
    }

    ArrayList<Block> blocks = pixy.getCCC().getBlocks(); // Gets a list of all blocks found by the Pixy2

    Block largestBlock = null;
    for (Block block : blocks) { // Loops through all blocks and finds the widest one
      if (largestBlock == null) {
        largestBlock = block;
      } else if (block.getWidth() > largestBlock.getWidth()) {
        largestBlock = block;
      }
    }
    return largestBlock;
  }

  @Override
  public void periodic() {

    // Gets Pixy data and stores in variables
    loadedBlock = this.getBiggestBlock();
    if (loadedBlock != null) {
      ballWidth = loadedBlock.getWidth();
      ballHeight = loadedBlock.getHeight();
      ballX = loadedBlock.getX();
      ballY = loadedBlock.getY();
      ballAngle = loadedBlock.getAngle();
      ballAge = loadedBlock.getAge();
    }


  
    // Uses the Pixy data to do things
    /*
     * if ( <button pressed> ) { this.driveTrain = driveTrain; this.shooter =
     * shooter; this.elevator = elevator; //take over drivetrain, elevator, and
     * shooter //stuff using above variables }
     */
  }

  public double getX() {
      return ballX;
  }

    public double getHeight() {
      return ballWidth;
    }

    public double getWidth() {
      return ballHeight;
    }

    public double getArea() {
      return ballWidth * ballHeight;
    }
}
