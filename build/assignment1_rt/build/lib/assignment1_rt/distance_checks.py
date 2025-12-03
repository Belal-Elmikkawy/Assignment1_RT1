import math


class TurtleChecks:
    @staticmethod
    def calculate_distance(pose1, pose2):
        """
        Calculates Euclidean distance between two turtle poses.
        """
        if pose1 is None or pose2 is None:
            return 0.0

        dx = pose1.x - pose2.x
        dy = pose1.y - pose2.y
        return math.sqrt(dx**2 + dy**2)

    @staticmethod
    def is_too_close(distance, threshold=1.5):
        """
        Returns True if turtles are closer than the threshold.
        """
        return distance < threshold

    @staticmethod
    def is_near_boundary(pose, min_limit=1.0, max_limit=10.0):
        """
        Returns True if the turtle is too close to the wall (1.0 or 10.0).
        """
        if pose is None:
            return False

        # Check X boundaries
        if pose.x < min_limit or pose.x > max_limit:
            return True
        # Check Y boundaries
        if pose.y < min_limit or pose.y > max_limit:
            return True

        return False
