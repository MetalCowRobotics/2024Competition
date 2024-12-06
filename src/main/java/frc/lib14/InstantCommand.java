package frc.lib14;

public class InstantCommand implements MCRCommand{
    boolean first = true;
    public void run(){
    }

    public boolean isFinished(){
		if(first){
            first = false;
            return false;
        }
        return true;
    }
}
