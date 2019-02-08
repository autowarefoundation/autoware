/*
 * Copyright 2015 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.design;

import com.ghostagent.SoundManagementActivity;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.LinearGradient;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.Shader;
import android.os.Handler;
import android.util.AttributeSet;
import android.view.View;

public class DrawCenterView extends View{
	int color;
	Paint paint;
	Canvas canvas;
	Shader topShader, bottomShader;
	Handler  mHandler   = new Handler();
	Rect rect;
	// parameters of rect
	int left, top, right, bottom;
	// number of rect
	int numberOfRect;
	// between blocks
	int space;
	// block's bulge
	int bulge = 1;
	// block size
	int widthSize, heightSize;

	public DrawCenterView(Context context, AttributeSet attrs) {
		super(context, attrs);
		color = Color.BLACK;
		paint = new Paint();
		paint.setAntiAlias(true);
		
		
		paint.setColor(color);

		left = SoundManagementActivity.viewWidth / 2 - widthSize;
		top = SoundManagementActivity.viewHeight / 2 + heightSize;
		right = SoundManagementActivity.viewWidth / 2 + widthSize;
		bottom = SoundManagementActivity.viewHeight / 2 - heightSize;

		rect = new Rect(left, top, right, bottom);
	}

	protected void onDraw(Canvas canvas){
		super.onDraw(canvas);
		int counter;

		//back color
		canvas.drawColor(Color.BLACK);

		if(SoundManagementActivity.getSizeFlag == true){
			//center rect
			//rect = new Rect(left, top, right, bottom);
			rect = new Rect(left, bottom, right, top);
			canvas.drawRect(rect, paint);

			for(counter = 2; counter <= numberOfRect; counter++){
				// set shader(top)
				paint.setShader(bottomShader);

				//upper rect
				//rect = new Rect(left, top + space * (counter - 1), right, bottom + space * (counter - 1));
				//rect = new Rect(left + bulge * counter, bottom + space * (counter - 1), right - bulge * counter, top + space * (counter - 1));
				rect = new Rect(left, bottom + space * (counter - 1), right, top + space * (counter - 1));
				canvas.drawRect(rect, paint);
				
				// set shader(bottom)
				paint.setShader(topShader);

				//under rect
				//rect = new Rect(left, top - space * (counter - 1), right, bottom - space * (counter - 1));
				//rect = new Rect(left + bulge * counter, bottom - space * (counter - 1), right - bulge * counter, top - space * (counter - 1));
				rect = new Rect(left, bottom - space * (counter - 1), right, top - space * (counter - 1));
				canvas.drawRect(rect, paint);
			}
		}
	}

	public void setColor(int color) {
		this.color = color;
	}

	public void drawView(int num){
		widthSize = SoundManagementActivity.viewWidth / 4;
		heightSize = SoundManagementActivity.viewHeight / 35;

		space = heightSize * 2 + 10;

		left = SoundManagementActivity.viewWidth / 2 - widthSize;
		top = SoundManagementActivity.viewHeight / 2 + heightSize;
		right = SoundManagementActivity.viewWidth / 2 + widthSize;
		bottom = SoundManagementActivity.viewHeight / 2 - heightSize;

		numberOfRect = num;
		
		paint.setColor(color);
		topShader = new LinearGradient(SoundManagementActivity.viewWidth, SoundManagementActivity.viewHeight,
					       SoundManagementActivity.viewWidth, 0,
					       color, color, Shader.TileMode.CLAMP);
		bottomShader = new LinearGradient(SoundManagementActivity.viewWidth, 0,
						  SoundManagementActivity.viewWidth, SoundManagementActivity.viewHeight,
						  color, color, Shader.TileMode.CLAMP);
		
		mHandler.post(new Runnable(){
			@Override
			public void run() {
				invalidate();		//to update the display
			}
		});
	}
}
