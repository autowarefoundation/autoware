//***********************************************************
// [SmartDK] SampleRoute
//
//【内　　容】
//	経路機能のサンプル
//
//【開発環境】
//	Android4.0以降を対象とする
//
// Copyright(C) 2014 INCREMENT P CORP.
//***********************************************************
package com.example.sampleroute;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;

import jp.incrementp.mapfan.smartdk.android.SmartDKManager;
import jp.incrementp.mapfan.smartdk.android.map.*;
import jp.incrementp.mapfan.smartdk.android.route.RouteGuideConnect;
import jp.incrementp.mapfan.smartdk.android.route.RouteGuidePoint;
import jp.incrementp.mapfan.smartdk.android.route.RouteResult;
import jp.incrementp.mapfan.smartdk.android.route.RouteResultList;
import jp.incrementp.mapfan.smartdk.android.route.RouteSearch;
import jp.incrementp.mapfan.smartdk.android.route.RouteSearchListener;
import jp.incrementp.mapfan.smartdk.android.utility.Utility;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.DialogInterface;
import android.os.Bundle;
import android.os.Environment;
import android.widget.Toast;
import android.app.FragmentManager;
import android.app.FragmentTransaction;
import android.graphics.Color;
import android.util.Log;

/**
 * 経路探索を実行するためのアクティビティ
 */
public class MainActivity extends SmartDKManager implements OnMapListener, OnGestureListener
{
	private MapFragment _mapFragment = null;
	private ProgressDialog _progressDialog = null;
	private RouteSearch _routeSearch = null;

	// ルート探索で利用する変数
	private boolean _isStartPointSet = false; // 出発地設定が完了していればtrue
	private boolean _isGoalPointSet = false;  // 目的地設定が完了していればtrue
	private boolean _isViaPointSet = false;   // 立寄地設定が完了しているばtrue
	private int _viaPointNum = 0; // 立寄地の数

	private boolean _isSearching = false; // ルート探索中かどうか

	/**
	 * アクティビティが初めて作成される際の処理
	 */
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		// 起動中の画面を表示する
		_progressDialog = new ProgressDialog(this);
		_progressDialog.setMessage("起動中");
		_progressDialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
		_progressDialog.setCancelable(false);
		_progressDialog.show();

		// 地図画面を表示するための準備
		FragmentManager fm = getFragmentManager();
		FragmentTransaction ft = fm.beginTransaction();

		// 既に地図画面が表示されているかどうかを確認する
		_mapFragment = (MapFragment)fm.findFragmentByTag("map");
		if (_mapFragment == null) {
			// まだ地図画面が表示されていない
			// →地図画面を新規作成する
			_mapFragment = new MapFragment();
		    ft.add(_mapFragment, "map");
		    // 地図画面を表示する
		    ft.commit();
		}

		// 地図の状態に変更があったときに呼び出すリスナーを登録する
		_mapFragment.setMapListener(this);
		_mapFragment.setGestureListener(this);
	}

	/**
	 * バックキーが押された際の処理
	 */
	@Override
	public void onBackPressed()
	{
		// 終了するかどうかを確認するためのダイアログを表示する
		new AlertDialog.Builder(getActivity())
		.setPositiveButton(R.string.finish, new DialogInterface.OnClickListener() {
			@Override
			public void onClick(DialogInterface dialog, int which) {
				// ルート探索機能の後処理をする
				if (_routeSearch != null) {
					_routeSearch.routeSearchFinish();
				}
				MainActivity.this.finish();
			}
		})
		.setNegativeButton(R.string.cancel, null)
		.setMessage(R.string.confirm_finish)
		.show();
	}

	/**
	 * 地図の中心地が変更された際の処理
	 */
	@Override
	public void onChangedMapCenter(Location location) {
	}

	/**
	 * 地図の角度が変更された際の処理
	 */
	@Override
	public void onChangedMapAngle(int angle) {
	}

	/**
	 * 地図の縮尺値が変更された際の処理
	 */
	@Override
	public void onChangedMapScale(long dispScale) {
	}

	/**
	 * 地図描画開始の際の処理
	 */
	@Override
	public void onStartDrawMap() {
	}

	/**
	 * 地図描画終了の際の処理
	 */
	@Override
	public void onEndDrawMap() {
	}

	/**
	 * 地図関係の処理でエラーが発生した際の処理
	 */
	@Override
	public void onError(int status) {
		new AlertDialog.Builder(getActivity()).setPositiveButton("OK",new DialogInterface.OnClickListener() {
			@Override
			public void onClick(DialogInterface dialog, int which) {
				// 通常終了
				MainActivity.this.finish();
			}
		})
		.setTitle("Error!!")
		.setMessage("エラーが発生しました。\nコード：" + status + "\nアプリを終了します。")
		.setCancelable(false)
		.show();
	}

	/**
	 * 1回タッチして次にもう一度タッチイベントが入らなかった際の処理
	 */
	@Override
	public void onSingleTapConfirmed(Location location) {
	}

	/**
	 * 2回タッチした際の処理
	 */
	@Override
	public void onDoubleTap(Location location) {
	}

	/**
	 * 長く押した際の処理
	 */
	@Override
	public void onLongPress(final Location location) {
		if (_routeSearch == null) {
			// ルートの初期化に失敗しているときは何も処理しない
			Toast.makeText(getActivity(), "ルートの初期化に失敗しています。", Toast.LENGTH_LONG).show();
			return;
		}

		// ルート設定用ダイアログを生成
		final RouteDialogFragment routeDialog = new RouteDialogFragment();
		// 以下、ボタンごとの処理
		routeDialog.setBtnStartListener(new RouteDialogFragment.BtnStartListener() {
			@Override
			public void onButton() {
				// 出発地を設定
				_isStartPointSet = _routeSearch.setStartingPoint(location.getLon(), location.getLat(), 0, RouteSearch.ROADTYPE_ALL);
				if (! _isStartPointSet) {
					new  AlertDialog.Builder(getActivity())
					.setTitle(R.string.error)
					.setMessage(R.string.route_point_invalid)
					.setPositiveButton(R.string.ok,  new DialogInterface.OnClickListener() {
						public  void onClick(DialogInterface dialog, int which) {
							dialog.cancel();
						}
					}).show();
				} else {
					// 出発地に点を表示
					if (_mapFragment.getGeometryCollection().isObject("p_start")) {
						_mapFragment.getGeometryCollection().remove("p_start");
					}
					PointOptions p_opt;	// 画像表示用のポイントオプション
					p_opt = new PointOptions();
					p_opt.addPosition(location);
					p_opt.setDotColor(Color.BLUE);
					p_opt.setSize(16);
					p_opt.setTitle(getString(R.string.start_point));
					p_opt.setVisible(true);
					_mapFragment.getGeometryCollection().addObject(p_opt, "p_start");
					_mapFragment.refresh();
				}
				routeDialog.dismiss();
			}
		});
		routeDialog.setBtnViaListener(new RouteDialogFragment.BtnViaListener() {
			@Override
			public void onButton() {
				// 経由地を更新
				// globalのArray型変数をリスト表示し、追加、削除が行えるようにする。
				_isViaPointSet = _routeSearch.setViaPoint(location.getLon(), location.getLat(), 0, RouteSearch.ROADTYPE_ALL);
				if (! _isViaPointSet) {
					new  AlertDialog.Builder(getActivity())
					.setTitle(R.string.error)
					.setMessage(R.string.route_point_invalid)
					.setPositiveButton(R.string.ok,  new DialogInterface.OnClickListener() {
						public  void onClick(DialogInterface dialog, int which) {
							dialog.cancel();
						}
					}).show();
				} else {
					// 目的地に点を表示
					PointOptions p_opt;	// 画像表示用のポイントオプション
					p_opt = new PointOptions();
					p_opt.addPosition(location);
					p_opt.setDotColor(Color.RED);
					p_opt.setSize(12);
					p_opt.setTitle(getString(R.string.via_point) + _viaPointNum);
					p_opt.setVisible(true);
					_mapFragment.getGeometryCollection().addObject(p_opt, ("p_via" +  _viaPointNum));
					_mapFragment.refresh();
					++_viaPointNum;
				}
				routeDialog.dismiss();
			}
		});
		routeDialog.setBtnGoalListener(new RouteDialogFragment.BtnGoalListener() {
			@Override
			public void onButton() {
				// 目的地を更新
				_isGoalPointSet = _routeSearch.setDestinationPoint(location.getLon(), location.getLat(), 0, RouteSearch.ROADTYPE_ALL);
				if (! _isGoalPointSet) {
					new  AlertDialog.Builder(getActivity())
					.setTitle(R.string.error)
					.setMessage(R.string.route_point_invalid)
					.setPositiveButton(R.string.ok,  new DialogInterface.OnClickListener() {
						public  void onClick(DialogInterface dialog, int which) {
							dialog.cancel();
						}
					}).show();
				} else {
					// 目的地に点を表示
					if(_mapFragment.getGeometryCollection().isObject("p_goal")) {
						_mapFragment.getGeometryCollection().remove("p_goal");
					}
					PointOptions p_opt;	//画像表示用のポイントオプション
					p_opt = new PointOptions();
					p_opt.addPosition(location);
					p_opt.setDotColor(Color.BLUE);
					p_opt.setSize(16);
					p_opt.setTitle(getString(R.string.goal_point));
					p_opt.setVisible(true);
					_mapFragment.getGeometryCollection().addObject(p_opt, "p_goal");
					_mapFragment.refresh();
				}
				routeDialog.dismiss();
			}
		});
		routeDialog.setBtnClearListener(new RouteDialogFragment.BtnClearListener() {
			@Override
			public void onButton() {
				// routeSearchの登録地点をクリアする
				// また、表示されているルートもクリアする
				new  AlertDialog.Builder(getActivity())
				.setTitle(R.string.route_setting)
				.setMessage(R.string.info_route_clear)
				.setPositiveButton(R.string.ok,  new DialogInterface.OnClickListener() {
					public  void onClick(DialogInterface dialog, int which) {
						// 検索処理中であればキャンセル
						if (_isSearching){
							_routeSearch.routeSearchCancel();
						}
						// 出発地を削除
						if(_mapFragment.getGeometryCollection().isObject("p_start")) {
							_mapFragment.getGeometryCollection().remove("p_start");
						}
						// 立寄地を削除
						for (int num = 0; num < _viaPointNum; ++num) {
							if(_mapFragment.getGeometryCollection().isObject("p_via" + num)) {
								_mapFragment.getGeometryCollection().remove("p_via" + num);
							}
						}
						// 目的地を削除
						if(_mapFragment.getGeometryCollection().isObject("p_goal")) {
							_mapFragment.getGeometryCollection().remove("p_goal");
						}
						// 描画されているルートをクリア
						_mapFragment.getRouteCollection().removeAll();
						// ルート情報をクリア
						if (_routeSearch != null) {
							_routeSearch.clearPoint();
						}
						_isStartPointSet = false;
						_isGoalPointSet = false;
						_viaPointNum = 0;
						_isSearching = false;
						routeDialog.setSearching(_isSearching);
						// 地図を再描画
						_mapFragment.refresh();
					}
				})
				.setNegativeButton(R.string.cancel,  new DialogInterface.OnClickListener() {
					public  void onClick(DialogInterface dialog, int which) {
						dialog.cancel();
					}
				}).show();
				routeDialog.dismiss();
			}
		});
		routeDialog.setBtnSearchListener(new RouteDialogFragment.BtnSearchListener() {
			@Override
			public void onButton() {
				// 出発地と目的地を確認
				if (! _isStartPointSet) {
					new  AlertDialog.Builder(getActivity())
					.setTitle(R.string.error)
					.setMessage(R.string.route_start_none)
					.setPositiveButton(R.string.ok,  new DialogInterface.OnClickListener() {
						public  void onClick(DialogInterface dialog, int which) {
							dialog.cancel();
						}
					}).show();
				} else if(!_isGoalPointSet) {
					new  AlertDialog.Builder(getActivity())
					.setTitle(R.string.error)
					.setMessage(R.string.route_goal_none)
					.setPositiveButton(R.string.ok,  new DialogInterface.OnClickListener() {
						public  void onClick(DialogInterface dialog, int which) {
							dialog.cancel();
						}
					}).show();
				} else {
					// ルート探索実行
					// 探索条件を指定する場合は、本関数をコールする前に設定する。
					_routeSearch.getRouteSimple(RouteSearch.SEARCH_TYPE_ROUTE, new RouteSearchListener() {
						@Override
						public void onRouteSearchResult(int searchStatus, RouteResult routeSearchResult) {
							_isSearching = false;
							// ルート探索完了後の処理
							if (searchStatus == RouteSearch.RESULT_OK) {
								// 結果を描画
								int count = routeSearchResult.getRouteResultCount();
								for (int i = 0; i < count; ++i) {
									String tag = "ROUTE" + i;
									if (_mapFragment.getRouteCollection().isObject(tag)) {
										// 既に引かれているルート線は消す
										_mapFragment.getRouteCollection().remove(tag);
									}
									RouteResultList resultList = routeSearchResult.getRouteResult(i);
									RouteCollection routeCollection = _mapFragment.getRouteCollection().addRouteObject(resultList, tag);
									routeCollection.setVisible(true);
									
									// 経路の出力
									Utility utility = new Utility();
									File file = new File(
											Environment.getExternalStorageDirectory().getPath() + "/MapRoute.txt");
									try {
										FileWriter filewriter = new FileWriter(file);
										BufferedWriter bw = new BufferedWriter(filewriter);
										PrintWriter pw = new PrintWriter(bw);
										
										for (RouteGuideConnect conn : resultList.mGuideConnect) {
											Log.w("SampleRoute", "size=" + conn.mGuidePoint.size());
											for (RouteGuidePoint pt : conn.mGuidePoint) {
												Location loc = new Location(pt.mLongitude, pt.mLatitude);
												double[] degree = utility.convert256ToDegree(loc);
												Log.w("SampleRoute", "  lat=" + degree[0] + ", lon=" + degree[1]);
												pw.println(degree[0] + "," + degree[1]);
											}
										}
										pw.close();
									} catch (Exception e) {
										e.printStackTrace();
									}
								}
								_mapFragment.refresh();
							} else if (searchStatus == RouteSearch.RESULT_SEARCH_CANCEL) {
								new AlertDialog.Builder(getActivity())
								.setPositiveButton("OK", null)
								.setMessage("ルート探索をキャンセルしました。")
								.show();
								// キャンセルしたため、RouteSearchを再初期化する
								_routeSearch.routeSearchInit();
							} else {
								String errorMessage = "";

								switch (searchStatus) {
								case RouteSearch.RESULT_FATAL_ERROR:
									errorMessage = getString(R.string.route_fatal_error);
									break;
								case RouteSearch.RESULT_ISOLATED_ISLAND:
									errorMessage = getString(R.string.route_isolated_island);
									break;
								case RouteSearch.RESULT_NO_TRANSIT_LINK:
									errorMessage = getString(R.string.route_no_transit_link);
									break;
								case RouteSearch.RESULT_TIME_REG:
									errorMessage = getString(R.string.route_time_reg);
									break;
								case RouteSearch.RESULT_VERSION_MISS_MATCH:
									errorMessage = getString(R.string.route_version_miss_match);
									break;
								default:
									errorMessage = getString(R.string.route_unknown_error);
									break;
								}

								new AlertDialog.Builder(getActivity())
								.setTitle(R.string.error)
								.setPositiveButton("OK", null)
								.setMessage(R.string.route_error_message + searchStatus + "\n" + errorMessage)
								.setCancelable(true)
								.show();

							}
						}
					});
				}
				routeDialog.dismiss();
				_isSearching = true;
			}
		});
		routeDialog.setBtnExitListener(new RouteDialogFragment.BtnExitListener() {
			@Override
			public void onButton() {
				routeDialog.dismiss();
				finish();
			}
		});

		routeDialog.setSearching(_isSearching);
		routeDialog.show(getFragmentManager(), "tag");
	}

	/**
	 * ドラック開始の処理
	 */
	@Override
	public void onDragStart(Location location) {
	}

	/**
	 * ドラック中の処理
	 */
	@Override
	public void onDragMove(Location location) {
	}

	/**
	 * ドラック終了の処理
	 */
	@Override
	public void onDragEnd(Location location) {
	}

	/**
	 * 地図描画が可能になった際の処理
	 */
	@Override
	public void onActivateView() {
		if (_progressDialog.isShowing()) {
			// 起動中の画面が表示されていたら消去する
			_progressDialog.dismiss();
		}
		// 地図の中心位置を名古屋大に設定
		Utility utility = new Utility();
		final double[] degreeval = {136.966458, 35.154329};
		_mapFragment.setCenterPosition(utility.convertDegreeTo256(degreeval), true);

		// RouteSearchクラスを初期化
		// RouteSerchクラスは、MapFragmentの初期化が完了した後に初期化する必要がある。
		if (_routeSearch == null) {
			_routeSearch = new RouteSearch(this);
			// 初期化処理
			// 出発地、目的地を指定する前に、RouteSearchクラスのrouteSearchInit()を必ずコールする必要がある。
			if (!_routeSearch.routeSearchInit()) {
				new  AlertDialog.Builder(getActivity())
				.setTitle(R.string.error)
				.setMessage(R.string.route_init_fail)
				.setPositiveButton(R.string.ok,  new DialogInterface.OnClickListener() {
					public  void onClick(DialogInterface dialog, int which) {
						dialog.cancel();
					}
				}).show();
				// routeSearchをnullに設定する
				_routeSearch = null;
			}
		}
	}

}
