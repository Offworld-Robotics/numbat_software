/*
 * Copyright (C) 2014 ros.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 *
 * Modified June 2014 for Bluesat OWR by Harry J.E Day
 */

package com.github.owr_gui.my_pub_sub_tutorial;

import java.util.*;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;
import javax.swing.*;
import java.awt.*;

/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class Listener extends AbstractNodeMain {
  
  
  JFrame FRAME;
  GPSPanel PANEL;
  int lastX = 0;
  int lastY = 0;
  int xOffset = 0;
  int yOffset = 0;
  final int frameSize = 500;
  
  public Listener() {
    super();
    FRAME = new JFrame("OWR GPS");  
    PANEL = new GPSPanel();
    FRAME.add(PANEL);
  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava/listener");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    FRAME.setSize(frameSize,frameSize);
    
    FRAME.setVisible(true);
    
    final Log log = connectedNode.getLog();
    Subscriber<sensor_msgs.NavSatFix> subscriber = connectedNode.newSubscriber("chatter", sensor_msgs.NavSatFix._TYPE);
    subscriber.addMessageListener(new MessageListener<sensor_msgs.NavSatFix>() {
      @Override
      public void onNewMessage(sensor_msgs.NavSatFix message) {
        doDrawing(message, log);
        log.info("I heard: \"" + message.toRawMessage().getFloat64("longitude") + "\"");
      }
    });
    while(true) {}
  }
  
  public void doDrawing(sensor_msgs.NavSatFix message, Log log) {
        
        
        
        
        PANEL.addCords( message.toRawMessage().getFloat64("longitude"), 
            message.toRawMessage().getFloat64("latitude"));
       
        
  }
  
  class GPSPanel extends JPanel {
    
    double scale = 0;
    int offsetX = 0;
    int offsetY = 0;
    final int SIZE = 500;
    final Stack<Double> LONGITUDES = new Stack<Double>();
    final Stack<Double> LATITUDES = new Stack<Double>();
    
    protected GPSPanel() {
        super();
        super.setSize(SIZE,SIZE);
        
    }

    public void paint(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.clearRect(0,0,SIZE,SIZE);
        //get copies of the stacks for drawing
        Stack<Double> x = (Stack) LONGITUDES.clone();
        Stack<Double> y = (Stack) LATITUDES.clone();
        
        if(!x.empty()) {
            double currentLong = x.peek();
            double currentLat = y.peek();
            g2d.drawString("current Longitude: " + currentLong + " current Lattitude" +
                currentLat, 10,10);
        }
        while(!x.empty()) {
            int x0 = (int) (x.pop()*scale) - offsetX;
            int y0 = (int) (y.pop()*scale) - offsetY;
            g2d.fillOval(x0,y0,5,5);
            System.out.println(x0 + "," + y0);
        }
        
        
    }
    
    public void addCords(double longitude, double latitude) {
            if(scale == 0) {
                
               /* int test = (int) longitude;
                while((test - (longitude * scale)) < 0) {
                    scale = scale*10;
                   
                    test = (int) (longitude*scale);
                }*/
                scale=100000;
                do  {
                    scale = scale/2;
                    offsetX = (int) (longitude*scale) - SIZE/2;
                    offsetY = (int) (latitude*scale) - SIZE/2;
                    System.out.println(scale);
                } while(((longitude * (double)scale) - offsetX) > SIZE);

                //scale = 10000;
                
                
            }
            //put them on the stack
            LONGITUDES.push(longitude);
            LATITUDES.push(latitude);
            
         
            this.paint(this.getGraphics());
    }
  }
}
