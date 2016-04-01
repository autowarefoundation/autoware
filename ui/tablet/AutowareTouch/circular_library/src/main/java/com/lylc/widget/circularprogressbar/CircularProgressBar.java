/*
 * Copyright 2013 Leon Cheng
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

package com.lylc.widget.circularprogressbar;

import android.animation.Animator;
import android.animation.Animator.AnimatorListener;
import android.animation.ObjectAnimator;
import android.animation.ValueAnimator;
import android.animation.ValueAnimator.AnimatorUpdateListener;
import android.content.Context;
import android.content.res.Resources;
import android.content.res.TypedArray;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Paint.Style;
import android.graphics.RectF;
import android.graphics.Typeface;
import android.text.TextUtils;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;
import android.view.animation.LinearInterpolator;
import android.widget.ProgressBar;
import com.lylc.widget.circularprogressbar.R;

public class CircularProgressBar extends ProgressBar{
	private static final String TAG = "CircularProgressBar";

	private static final int STROKE_WIDTH = 20;

	private String mTitle = "";		
	private String mSubTitle = "";

	private int mStrokeWidth = STROKE_WIDTH;

	private final RectF mCircleBounds = new RectF();

	private final Paint mProgressColorPaint = new Paint();
	private final Paint mBackgroundColorPaint = new Paint();
	private final Paint mTitlePaint = new Paint(); 
	private final Paint mSubtitlePaint = new Paint();

	private boolean mHasShadow = true;
	private int mShadowColor = Color.BLACK;

	public interface ProgressAnimationListener{
		public void onAnimationStart();
		public void onAnimationFinish();
		public void onAnimationProgress(int progress);
	}

	public CircularProgressBar(Context context) {
		super(context);
		init(null, 0);
	}

	public CircularProgressBar(Context context, AttributeSet attrs) {
		super(context, attrs);
		init(attrs, 0);
	}

	public CircularProgressBar(Context context, AttributeSet attrs, int defStyle) {
		super(context, attrs, defStyle);
		init(attrs, defStyle);
	}

	public void init(AttributeSet attrs, int style){
		//so that shadow shows up properly for lines and arcs
		setLayerType(View.LAYER_TYPE_SOFTWARE, null);

		TypedArray a = getContext().obtainStyledAttributes(attrs,
				R.styleable.CircularProgressBar, style, 0);

		String color;
		Resources res = getResources();

		this.mHasShadow = a.getBoolean(R.styleable.CircularProgressBar_cpb_hasShadow, true);

		color = a.getString(R.styleable.CircularProgressBar_cpb_progressColor);
		if(color==null)
			mProgressColorPaint.setColor(res.getColor(R.color.circular_progress_default_progress));
		else
			mProgressColorPaint.setColor(Color.parseColor(color));

		color = a.getString(R.styleable.CircularProgressBar_cpb_backgroundColor);
		if(color==null)
			mBackgroundColorPaint.setColor(res.getColor(R.color.circular_progress_default_background));
		else
			mBackgroundColorPaint.setColor(Color.parseColor(color));

		color = a.getString(R.styleable.CircularProgressBar_cpb_titleColor);
		if(color==null)
			mTitlePaint.setColor(res.getColor(R.color.circular_progress_default_title));
		else
			mTitlePaint.setColor(Color.parseColor(color));

		color = a.getString(R.styleable.CircularProgressBar_cpb_subtitleColor);
		if(color==null)
			mSubtitlePaint.setColor(res.getColor(R.color.circular_progress_default_subtitle));
		else
			mSubtitlePaint.setColor(Color.parseColor(color));


		String t = a.getString(R.styleable.CircularProgressBar_cpb_title);
		if(t!=null)
			mTitle = t;

		t = a.getString(R.styleable.CircularProgressBar_cpb_subtitle);
		if(t!=null)
			mSubTitle = t;

		mStrokeWidth = a.getInt(R.styleable.CircularProgressBar_cpb_strokeWidth, STROKE_WIDTH);

		a.recycle();


		mProgressColorPaint.setAntiAlias(true);
		mProgressColorPaint.setStyle(Paint.Style.STROKE);
		mProgressColorPaint.setStrokeWidth(mStrokeWidth);

		mBackgroundColorPaint.setAntiAlias(true);
		mBackgroundColorPaint.setStyle(Paint.Style.STROKE);
		mBackgroundColorPaint.setStrokeWidth(mStrokeWidth);

		mTitlePaint.setTextSize(60); 
		mTitlePaint.setStyle(Style.FILL);
		mTitlePaint.setAntiAlias(true);
		mTitlePaint.setTypeface(Typeface.create("Roboto-Thin", Typeface.NORMAL));
		mTitlePaint.setShadowLayer(0.1f, 0, 1, Color.GRAY);

		mSubtitlePaint.setTextSize(20); 
		mSubtitlePaint.setStyle(Style.FILL);
		mSubtitlePaint.setAntiAlias(true);
		mSubtitlePaint.setTypeface(Typeface.create("Roboto-Thin", Typeface.BOLD));
		//		mSubtitlePaint.setShadowLayer(0.1f, 0, 1, Color.GRAY);
	}

	@Override
	protected synchronized void onDraw(Canvas canvas) {
		canvas.drawArc(mCircleBounds, 0, 360 , false, mBackgroundColorPaint);

		int prog = getProgress();
		float scale = getMax() > 0 ? (float)prog/getMax() *360: 0;

		if(mHasShadow)
			mProgressColorPaint.setShadowLayer(	3, 0, 1, mShadowColor);
		canvas.drawArc(mCircleBounds, 270, scale , false, mProgressColorPaint);


		if(!TextUtils.isEmpty(mTitle)){
			int xPos =  (int)(getMeasuredWidth()/2 - mTitlePaint.measureText(mTitle) / 2);
			int yPos = (int) (getMeasuredHeight()/2);

			float titleHeight = Math.abs(mTitlePaint.descent() + mTitlePaint.ascent());
			if(TextUtils.isEmpty(mSubTitle)){
				yPos += titleHeight/2;
			}
			canvas.drawText(mTitle, xPos, yPos, mTitlePaint); 

			yPos += titleHeight;
			xPos = (int)(getMeasuredWidth()/2 - mSubtitlePaint.measureText(mSubTitle) / 2);

			canvas.drawText(mSubTitle, xPos, yPos, mSubtitlePaint);
		}		

		super.onDraw(canvas);
	}

	@Override
	protected void onMeasure(final int widthMeasureSpec, final int heightMeasureSpec) {
		final int height = getDefaultSize(getSuggestedMinimumHeight(), heightMeasureSpec);
		final int width = getDefaultSize(getSuggestedMinimumWidth(), widthMeasureSpec);
		final int min = Math.min(width, height);
		setMeasuredDimension(min+2*STROKE_WIDTH, min+2*STROKE_WIDTH);

		mCircleBounds.set(STROKE_WIDTH, STROKE_WIDTH, min+STROKE_WIDTH, min+STROKE_WIDTH);
	}

	@Override
	public synchronized void setProgress(int progress) {
		super.setProgress(progress);

		// the setProgress super will not change the details of the progress bar
		// anymore so we need to force an update to redraw the progress bar
		invalidate();
	}

	public void animateProgressTo(final int start, final int end, final ProgressAnimationListener listener){
		if(start!=0)
			setProgress(start);

		final ObjectAnimator progressBarAnimator = ObjectAnimator.ofFloat(this, "animateProgress", start, end);
		progressBarAnimator.setDuration(1500);
		//		progressBarAnimator.setInterpolator(new AnticipateOvershootInterpolator(2f, 1.5f));
		progressBarAnimator.setInterpolator(new LinearInterpolator());

		progressBarAnimator.addListener(new AnimatorListener() {
			@Override
			public void onAnimationCancel(final Animator animation) {
			}

			@Override
			public void onAnimationEnd(final Animator animation) {
				CircularProgressBar.this.setProgress(end);
				if(listener!=null)
					listener.onAnimationFinish();
			}

			@Override
			public void onAnimationRepeat(final Animator animation) {
			}

			@Override
			public void onAnimationStart(final Animator animation) {
				if(listener!=null)
					listener.onAnimationStart();
			}
		});

		progressBarAnimator.addUpdateListener(new AnimatorUpdateListener() {
			@Override
			public void onAnimationUpdate(final ValueAnimator animation) {
				int progress = ((Float) animation.getAnimatedValue()).intValue();
				if(progress!=CircularProgressBar.this.getProgress()){
					Log.d(TAG, progress + "");
					CircularProgressBar.this.setProgress(progress);
					if(listener!=null)
						listener.onAnimationProgress(progress);					
				}
			}
		});
		progressBarAnimator.start();
	}

	public synchronized void setTitle(String title){
		this.mTitle = title;
		invalidate();
	}

	public synchronized void setSubTitle(String subtitle){
		this.mSubTitle = subtitle;
		invalidate();
	}

	public synchronized void setSubTitleColor(int color){
		mSubtitlePaint.setColor(color);
		invalidate();
	}

	public synchronized void setTitleColor(int color){
		mTitlePaint.setColor(color);
		invalidate();
	}

	public synchronized void setHasShadow(boolean flag){
		this.mHasShadow = flag;
		invalidate();
	}

	public synchronized void setShadow(int color){
		this.mShadowColor = color;
		invalidate();
	}

	public String getTitle(){
		return mTitle;
	}

	public boolean getHasShadow(){
		return mHasShadow;
	}
}
