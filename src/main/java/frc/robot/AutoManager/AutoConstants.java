package frc.robot.AutoManager;

public class AutoConstants {

    public static class AutoPaths {

		public static class PathType {

			String startingLocation;
			String endingLocation;

			private PathType(String startingLocation, String endingLocation) {
				this.startingLocation = startingLocation;
				this.endingLocation = endingLocation;
			}

			public PathType from(String startingLocation, String endingLocation) {
				return new PathType(startingLocation, endingLocation);
			}
		}
    }
}
