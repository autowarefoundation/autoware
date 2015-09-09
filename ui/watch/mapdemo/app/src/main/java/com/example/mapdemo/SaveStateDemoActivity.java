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
import com.google.android.gms.maps.GoogleMap.OnMarkerClickListener;
import com.google.android.gms.maps.GoogleMap.OnMarkerDragListener;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;

import android.os.Bundle;
import android.os.Parcel;
import android.os.Parcelable;
import android.support.v4.app.FragmentActivity;

import java.util.Random;

/**
 * This activity shows how to save the state of a MapFragment when the activity is recreated, like
 * after rotation of the device.
 */
public class SaveStateDemoActivity extends FragmentActivity {

    /** Default marker position when the activity is first created. */
    private static final LatLng DEFAULT_MARKER_POSITION = new LatLng(48.858179, 2.294576);

    /** List of hues to use for the marker */
    private static final float[] MARKER_HUES = new float[]{
        BitmapDescriptorFactory.HUE_RED,
        BitmapDescriptorFactory.HUE_ORANGE,
        BitmapDescriptorFactory.HUE_YELLOW,
        BitmapDescriptorFactory.HUE_GREEN,
        BitmapDescriptorFactory.HUE_CYAN,
        BitmapDescriptorFactory.HUE_AZURE,
        BitmapDescriptorFactory.HUE_BLUE,
        BitmapDescriptorFactory.HUE_VIOLET,
        BitmapDescriptorFactory.HUE_MAGENTA,
        BitmapDescriptorFactory.HUE_ROSE,
    };

    // Bundle keys.
    private static final String OTHER_OPTIONS = "options";
    private static final String MARKER_POSITION = "markerPosition";
    private static final String MARKER_INFO = "markerInfo";

    /**
     * Extra info about a marker.
     */
    static class MarkerInfo implements Parcelable {

        public static final Parcelable.Creator<MarkerInfo> CREATOR =
                new Parcelable.Creator<MarkerInfo>() {
                    @Override
                    public MarkerInfo createFromParcel(Parcel in) {
                        return new MarkerInfo(in);
                    }

                    @Override
                    public MarkerInfo[] newArray(int size) {
                        return new MarkerInfo[size];
                    }
                };

        float mHue;

        public MarkerInfo(float color) {
            mHue = color;
        }

        private MarkerInfo(Parcel in) {
            mHue = in.readFloat();
        }

        @Override
        public int describeContents() {
            return 0;
        }

        @Override
        public void writeToParcel(Parcel dest, int flags) {
            dest.writeFloat(mHue);
        }
    }

    /**
     * Example of a custom {@code MapFragment} showing how the position of a marker and other custom
     * {@link Parcelable}s objects can be saved after rotation of the device.
     * <p>
     * Storing custom {@link Parcelable} objects directly in the {@link Bundle} provided by the
     * {@link #onActivityCreated(Bundle)} method will throw a {@code ClassNotFoundException}. This
     * is due to the fact that this Bundle is parceled (thus losing its ClassLoader attribute at
     * this moment) and unparceled later in a different ClassLoader.
     * <br>
     * A workaround to store these objects is to wrap the custom {@link Parcelable} objects in a new
     * {@link Bundle} object.
     * <p>
     * However, note that it is safe to store {@link Parcelable} objects from the Maps API (eg.
     * MarkerOptions, LatLng, etc.) directly in the Bundle provided by the
     * {@link #onActivityCreated(Bundle)} method.
     */
    public static class SaveStateMapFragment extends SupportMapFragment
            implements OnMarkerClickListener, OnMarkerDragListener, OnMapReadyCallback {

        private LatLng mMarkerPosition;
        private MarkerInfo mMarkerInfo;
        private boolean mMoveCameraToMarker;


        @Override
        public void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);

            if (savedInstanceState == null) {
                // Activity created for the first time.
                mMarkerPosition = DEFAULT_MARKER_POSITION;
                mMarkerInfo = new MarkerInfo(BitmapDescriptorFactory.HUE_RED);
                mMoveCameraToMarker = true;
            } else {
                // Extract the state of the MapFragment:
                // - Objects from the API (eg. LatLng, MarkerOptions, etc.) were stored directly in
                //   the savedInsanceState Bundle.
                // - Custom Parcelable objects were wrapped in another Bundle.

                mMarkerPosition = savedInstanceState.getParcelable(MARKER_POSITION);

                Bundle bundle = savedInstanceState.getBundle(OTHER_OPTIONS);
                mMarkerInfo = bundle.getParcelable(MARKER_INFO);

                mMoveCameraToMarker = false;
            }

            getMapAsync(this);
        }


        @Override
        public void onSaveInstanceState(Bundle outState) {
            super.onSaveInstanceState(outState);

            // All Parcelable objects of the API  (eg. LatLng, MarkerOptions, etc.) can be set
            // directly in the given Bundle.
            outState.putParcelable(MARKER_POSITION, mMarkerPosition);

            // All other custom Parcelable objects must be wrapped in another Bundle. Indeed,
            // failing to do so would throw a ClassNotFoundException. This is due to the fact that
            // this Bundle is being parceled (losing its ClassLoader at this time) and unparceled
            // later in a different ClassLoader.
            Bundle bundle = new Bundle();
            bundle.putParcelable(MARKER_INFO, mMarkerInfo);
            outState.putBundle(OTHER_OPTIONS, bundle);
        }

        @Override
        public boolean onMarkerClick(Marker marker) {
            float newHue = MARKER_HUES[new Random().nextInt(MARKER_HUES.length)];
            mMarkerInfo.mHue = newHue;
            marker.setIcon(BitmapDescriptorFactory.defaultMarker(newHue));
            return true;
        }

        @Override
        public void onMapReady(GoogleMap map) {
            MarkerOptions markerOptions = new MarkerOptions()
                .position(mMarkerPosition)
                .icon(BitmapDescriptorFactory.defaultMarker(mMarkerInfo.mHue))
                .draggable(true);
            map.addMarker(markerOptions);
            map.setOnMarkerDragListener(this);
            map.setOnMarkerClickListener(this);

            if (mMoveCameraToMarker) {
                map.animateCamera(CameraUpdateFactory.newLatLng(mMarkerPosition));
            }
        }

        @Override
        public void onMarkerDragStart(Marker marker) {}

        @Override
        public void onMarkerDrag(Marker marker) {}

        @Override
        public void onMarkerDragEnd(Marker marker) {
            mMarkerPosition = marker.getPosition();
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.save_state_demo);
    }
}
