package edu.umich.soarrobot.SoarRobotTablet.layout;

import edu.umich.soarrobot.SoarRobotTablet.SoarRobotTablet;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimObject;

public interface IMapView {

	void setNextClass(CharSequence charSequence);

	void zoomOut();
	void zoomIn();

	void setActivity(SoarRobotTablet soarRobotTablet);

	void draw();

	void addObject(SimObject sim);
	void addRobot(SimObject robot);
	void pickUpObject(String robotName, int id);

	void deserializeMap(String substring);

	SimObject getRobot(String string);

	void dropObject(String name);

	SimObject getSimObject(int id);

	void doorClose(int id);

	void doorOpen(int id);

	void roomLight(int id, boolean on);

}
