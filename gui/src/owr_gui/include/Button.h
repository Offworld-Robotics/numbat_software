/* Text Labelled Buttons for GUIs for Off World Robotics 
 *
 *
 */


class Button {
   private:
	double posX;
	double posY;
	double xHalfLen;
	double yHalfLen;
	float R, G, B;
	char label[10];
	bool isClicked;
	
   public:
	void draw();
	Button(double x, double y, double width, double height, float r, float g, float b, char *txt);
	void changeColour(float r, float g, float b);
	bool isOnButton(int x, int y);
	void click();
	void unclick();
	double getX();
	double getY();
	void setPosition(double x, double y);
};
