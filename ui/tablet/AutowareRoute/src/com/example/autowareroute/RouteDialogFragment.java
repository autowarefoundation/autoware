//***********************************************************
// [SmartDK] AutowareRoute
//
//【内　　容】
//	ポップアップメニュー用のフラグメント
//
//【開発環境】
//	Android4.0以降を対象とする
//
// Copyright(C) 2014 INCREMENT P CORP.
//***********************************************************
package com.example.autowareroute;


import android.app.DialogFragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

/**
 * 経路探索クラスを操作するフラグメント
 */
public class RouteDialogFragment extends DialogFragment {
	private boolean _searching = false; // ルート探索中かどうか

	/**
	 * フラグメントのビュー作成時の処理
	 */
	@Override
	public View onCreateView(final LayoutInflater inflater,
			final ViewGroup container,
			final Bundle savedInstanceState) {
		getDialog().setTitle(R.string.route_setting);

		View view = inflater.inflate(R.layout.custom_dialog, container, false);

		// ルート探索中はボタンを押せないようにする
		Button button = (Button)view.findViewById(R.id.BtnSearch);
		if (_searching) {
			button.setEnabled(false);
		} else {
			button.setEnabled(true);
		}

		return view;
	}

	/**
	 * フラグメント作成時の処理
	 */
	@Override
	public void onActivityCreated(final Bundle savedInstanceState) {
		super.onActivityCreated(savedInstanceState);
		getView().findViewById(R.id.BtnStart).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if (btnStartListener != null){
					btnStartListener.onButton();
				}
			}
		});
		getView().findViewById(R.id.BtnVia).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if (btnViaListener != null){
					btnViaListener.onButton();
				}
			}
		});
		getView().findViewById(R.id.BtnSGoal).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if (btnGoalListener != null){
					btnGoalListener.onButton();
				}
			}
		});
		getView().findViewById(R.id.BtnClear).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if (btnClearListener != null){
					btnClearListener.onButton();
				}
			}
		});
		getView().findViewById(R.id.BtnSearch).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if (btnSearchListener != null){
					btnSearchListener.onButton();
				}
			}
		});
		getView().findViewById(R.id.BtnExit).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if (btnExitListener != null){
					btnExitListener.onButton();
				}
			}
		});

	}

	// 出発地に設定ボタンがタップされた時の処理
	private BtnStartListener btnStartListener;
	public interface BtnStartListener {
		public void onButton();
	}
	public void setBtnStartListener(BtnStartListener btnStartListener) {
		this.btnStartListener = btnStartListener;
	}

	// 立寄地に設定ボタンがタップされた時の処理
	private BtnViaListener btnViaListener;
	public interface BtnViaListener {
		public void onButton();
	}
	public void setBtnViaListener(BtnViaListener btnViaListener) {
		this.btnViaListener = btnViaListener;
	}

	// 目的地に設定ボタンがタップされた時の処理
	private BtnGoalListener btnGoalListener;
	public interface BtnGoalListener {
		public void onButton();
	}
	public void setBtnGoalListener(BtnGoalListener btnGoalListener) {
		this.btnGoalListener = btnGoalListener;
	}

	// ルート探索実行ボタンがタップされた時の処理
	private BtnSearchListener btnSearchListener;
	public interface BtnSearchListener {
		public void onButton();
	}
	public void setBtnSearchListener(BtnSearchListener btnSearchListener) {
		this.btnSearchListener= btnSearchListener;
	}

	// 終了ボタンがタップされた時の処理
	private BtnExitListener btnExitListener;
	public interface BtnExitListener {
		public void onButton();
	}
	public void setBtnExitListener(BtnExitListener btnExitListener) {
		this.btnExitListener= btnExitListener;
	}

	// ルート消去ボタンがタップされた時の処理
	private BtnClearListener btnClearListener;
	public interface BtnClearListener {
		public void onButton();
	}
	public void setBtnClearListener(BtnClearListener btnClearListener) {
		this.btnClearListener = btnClearListener;
	}

	// ルート探索中かどうかの値を外部から設定する
	public void setSearching(boolean searching) {
		this._searching = searching;
	}
}
