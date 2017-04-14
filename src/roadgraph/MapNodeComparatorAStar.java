package roadgraph;

import java.util.Comparator;

public class MapNodeComparatorAStar implements Comparator<MapNode> {
    @Override
    public int compare(MapNode x, MapNode y)
    {
		if(x.getSearchDistance() + x.getEstimatedDistanceFromGoal() < y.getSearchDistance() + y.getEstimatedDistanceFromGoal()) return -1;
		if(x.getSearchDistance() + x.getEstimatedDistanceFromGoal() > y.getSearchDistance() + y.getEstimatedDistanceFromGoal()) return 1;
		return 0;
    }
}
