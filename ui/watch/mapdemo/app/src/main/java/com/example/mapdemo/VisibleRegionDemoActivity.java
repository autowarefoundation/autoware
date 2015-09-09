/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.mapdemo;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.GoogleMap.OnCameraChangeListener;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.CameraPosition;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.MarkerOptions;

import android.os.Bundle;
import android.os.Handler;
import android.os.SystemClock;
import android.support.v4.app.FragmentActivity;
import android.view.View;
import android.view.animation.Interpolator;
import android.view.animation.OvershootInterpolator;
import android.widget.TextView;
import android.widget.Toast;

/**
 * This shows how to use setPadding to allow overlays that obscure part of the map without
 * obscuring the map UI or copyright notices.
 */
public class VisibleRegionDemoActivity extends FragmentActivity implements OnMapReadyCallback {
    /**
     * Note that this may be null if the Google Play services APK is not available.
     */
    private GoogleMap mMap;

    private static final LatLng SOH = new LatLng(-33.85704, 151.21522);
    private static final LatLng SFO = new LatLng(37.614631, -122.385153);
    private static final LatLngBounds AUS = new LatLngBounds(
            new LatLng(-44, 113), new LatLng(-10, 154));

    private TextView mMessageView;

    /** Keep track of current values for padding, so we can animate from them. */
    int currentLeft = 150;
    int currentTop = 0;
    int currentRight = 0;
    int currentBottom = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.visible_region_demo);
        mMessageView = (TextView) findViewById(R.id.message_text);

        SupportMapFragment mapFragment =
                (SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);
    }

    @Override
    public void onMapReady(GoogleMap map) {
        mMap = map;

        // turn MyLocation on and move to a place with indoor (SFO airport)
        mMap.setMyLocationEnabled(true);
        mMap.setPadding(currentLeft, currentTop, currentRight, currentBottom);
        mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(SFO, 18));
        // Add a marker to the Opera House
        mMap.addMarker(new MarkerOptions().position(SOH).title("Sydney Opera House"));
        // Add a camera change listener.
        mMap.setOnCameraChangeListener(new OnCameraChangeListener() {
            @Override
            public void onCameraChange(CameraPosition pos) {
                mMessageView.setText("CameraChangeListener: " + pos);
            }
        });
    }

    /**
     * Checks if the map is ready (which depends on whether the Google Play services APK is
     * available. This should be called prior to calling any methods on GoogleMap.
     */
    private boolean checkReady() {
        if (mMap == null) {
            Toast.makeText(this, R.string.map_not_ready, Toast.LENGTH_SHORT).show();
            return false;
        }
        return true;
    }

    public void moveToOperaHouse(View view) {
        if (!checkReady()) {
            return;
        }
        mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(SOH, 16));
    }

    public void moveToSFO(View view) {
        if (!checkReady()) {
            return;
        }
        mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(SFO, 18));
    }

    public void moveToAUS(View view) {
        if (!checkReady()) {
            return;
        }
        mMap.moveCamera(CameraUpdateFactory.newLatLngBounds(AUS, 0));
    }

    public void setNoPadding(View view) {
        if (!checkReady()) {
            return;
        }
        animatePadding(150, 0, 0, 0);
    }

    public void setMorePadding(View view) {
        if (!checkReady()) {
            return;
        }
        View mapView = ((SupportMapFragment)
            getSupportFragmentManager().findFragmentById(R.id.map)).getView();
        int left = 150;
        int top = 0;
        int right = mapView.getWidth() / 3;
        int bottom = mapView.getHeight() / 4;
        animatePadding(left, top, right, bottom);
    }

    public void animatePadding(
        final int toLeft, final int toTop, final int toRight, final int toBottom) {

        final Handler handler = new Handler();
        final long start = SystemClock.uptimeMillis();
        final long duration = 1000;

        final Interpolator interpolator = new OvershootInterpolator();

        final int startLeft = currentLeft;
        final int startTop = currentTop;
        final int startRight = currentRight;
        final int startBottom = currentBottom;

        currentLeft = toLeft;
        currentTop = toTop;
        currentRight = toRight;
        currentBottom = toBottom;

        handler.post(new Runnable() {
            @Override
            public void run() {
                long elapsed = SystemClock.uptimeMillis() - start;
                float t = interpolator.getInterpolation((float) elapsed / duration);

                int left = (int) (startLeft + ((toLeft - startLeft) * t));
                int top = (int) (startTop + ((toTop - startTop) * t));
                int right = (int) (startRight + ((toRight - startRight) * t));
                int bottom = (int) (startBottom + ((toBottom - startBottom) * t));

                mMap.setPadding(left, top, right, bottom);

                if (elapsed < duration) {
                    // Post again 16ms later.
                    handler.postDelayed(this, 16);
                }
            }
        });
    }
}
