/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

public class DrawRightView extends View{
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

	public DrawRightView(Context context, AttributeSet attrs) {
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
					       SoundManagementActivity.viewWidth, SoundManagementActivity.viewHeight / 8,
					       color, color, Shader.TileMode.CLAMP);
		bottomShader = new LinearGradient(SoundManagementActivity.viewWidth, 0,
						  SoundManagementActivity.viewWidth, SoundManagementActivity.viewHeight - SoundManagementActivity.viewHeight / 8,
						  color, color, Shader.TileMode.CLAMP);

		mHandler.post(new Runnable(){
			@Override
			public void run() {
				invalidate();		//to update the display
			}
		});
	}
}
