package frc.lib14;

import java.util.ArrayList;

public class ParallelCommands implements MCRCommand {
	private ArrayList<MCRCommand> parallelCommands = new ArrayList<MCRCommand>();
	
	public ParallelCommands(MCRCommand... commands) {
		for(MCRCommand command : commands) {
			parallelCommands.add(command);
		}
	}

	@Override
	public void run() {
		for (MCRCommand command : parallelCommands) {
			if (!command.isFinished()) {
				command.run();
			}
		}
	}

	@Override
	public boolean isFinished() {
		for (MCRCommand command : parallelCommands) {
			if (!command.isFinished()) {
				return false;
			}
		}
		return true;
	}

}