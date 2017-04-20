package roadgraph;

import java.util.Comparator;

public class MapNodeComparatorPathTime implements Comparator<MapNode> {
    @Override
    public int compare(MapNode x, MapNode y)
    {
		if(x.getPathTime() + x.getEstimatedTimeFromGoal() < y.getPathTime() + y.getEstimatedTimeFromGoal()) return -1;
		if(x.getPathTime() + x.getEstimatedTimeFromGoal() > y.getPathTime() + y.getEstimatedTimeFromGoal()) return 1;
		return 0;
    }
}