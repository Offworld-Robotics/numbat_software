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

#include <string>

class Button {
   private:
	double x;
	double y;
	double halfWidth;
	double halfHeight;
	float r, g, b;
	std::string label;
	bool isClicked;
	void (*downFunc)(void);
	void (*upFunc)(void);
	void init(double x, double y, double width, double height, float r, float g, float b, char *txt, void (*downFunc)(void), void (*upFunc)(void));
	Button();
	
   public:
	Button(double x, double y, double width, double height, float r, float g, float b, char *txt, void (*downFunc)(void), void (*upFunc)(void));
	void draw();
	void setColour(float r, float g, float b);
	bool isInside(int x, int y);
	void click();
	void unclick();
	double getX();
	double getY();
	void setPosition(double x, double y);
	void setDownFunc(void (*downFunc)(void));
	void setUpFunc(void (*upFunc)(void));
        // override these in derived classes
        virtual void clickDownOperation();
        virtual void clickUpOperation();
};

#endif // BUTTON_H
