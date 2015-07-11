#ifndef BUTTON_H
#define BUTTON_H

/* Text Labelled Buttons for GUIs for Off World Robotics 
 *
 * These need to be subclassed and 
 *    clickDownOperation()
 *    clickUpOperation()
 *    overridden with needed funcionality for each button
 *       base implementation does nothing
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
	Button();
	Button(double x, double y, double width, double height, float r, float g, float b, char *txt);
	void changeColour(float r, float g, float b);
	bool isPointInBounds(int x, int y);
	void click();
	void unclick();
	double getX();
	double getY();
	void setPosition(double x, double y);
        // override these in derived classes
        virtual void clickDownOperation();
        virtual void clickUpOperation();
};

#endif // BUTTON_H
