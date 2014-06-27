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
  int lastX = 0;
  int lastY = 0;
  int xOffset = 0;
  int yOffset = 0;
  final int frameSize = 500;
  
  public Listener() {
    super();
    FRAME = new JFrame("OWR GPS");  
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
        int x = (int) (message.toRawMessage().getFloat64("longitude")*100);
        int y = (int) (message.toRawMessage().getFloat64("latitude")*100);
        if(yOffset == 0) {
            //centre first point
            yOffset = y - frameSize/2;
            xOffset = x - frameSize/2;
            lastX = x;
            lastY = y;
            
        }
        
        Graphics2D g =(Graphics2D) FRAME.getGraphics();
        log.info("x1:" + (lastX - xOffset) + "y1:" + (lastY - yOffset) + "x2:" + (x - xOffset) + "y2" +  (y - yOffset));
        g.drawLine(lastX - xOffset,lastY - yOffset,x - xOffset,y - yOffset);
        
  }
}
