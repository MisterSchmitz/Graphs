package roadgraph;

import java.util.Comparator;

public class DijkstraComparator implements Comparator<MapNode> {
	
	@Override
	public int compare(MapNode a, MapNode b) {
		if(a.getSearchDistance() < b.getSearchDistance()) return -1;
		if(a.getSearchDistance() > b.getSearchDistance()) return 1;
		return 0;
	}

}
