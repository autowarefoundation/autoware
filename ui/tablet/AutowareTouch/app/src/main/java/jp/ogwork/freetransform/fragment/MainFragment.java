package jp.ogwork.freetransform.fragment;

import static android.content.Intent.ACTION_PICK;
import static android.provider.MediaStore.Images.Media.EXTERNAL_CONTENT_URI;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;

import jp.ogwork.freetransform.R;
import jp.ogwork.gesturetransformableview.view.GestureTransformableImageView;
import android.app.Activity;
import android.app.Fragment;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Bitmap.CompressFormat;
import android.graphics.drawable.Drawable;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ImageView.ScaleType;
import android.widget.RelativeLayout;
import android.widget.RelativeLayout.LayoutParams;
import android.widget.Toast;

import com.squareup.picasso.Picasso;

public class MainFragment extends Fragment {

    private static final int GALLERY_REQUEST = 0x0001;

    private static final int GALLERY_REQUEST_BG = 0x0002;

    private static final int IMAGE_VIEW_WIDTH = 600;

    private static final int IMAGE_VIEW_HEIGHT = 600;

    private static final int LOAD_IMAGE_RESIZE_WIDTH = 500;

    private static final int LOAD_IMAGE_RESIZE_HEIGHT = 500;

    private static final String SAVED_CAPTURE_IMAGE_NAME = "capture.png";

    private static final int WC = ViewGroup.LayoutParams.WRAP_CONTENT;

    private ArrayList<GestureTransformableImageView> imageViewList = new ArrayList<GestureTransformableImageView>();

    private Button btnAddView;

    private Button btnAddBg;

    private Button btnCapture;

    public MainFragment() {
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {

        View rootView = inflater.inflate(R.layout.fragment_main, container, false);

        btnAddView = (Button) rootView.findViewById(R.id.btn_add_view);
        btnAddView.setOnClickListener(new OnClickListener() {

            @Override
            public void onClick(View v) {
                if (imageViewList != null) {
                    Intent gallery = new Intent(ACTION_PICK, EXTERNAL_CONTENT_URI);
                    startActivityForResult(gallery, GALLERY_REQUEST);
                }
            }
        });

        btnAddBg = (Button) rootView.findViewById(R.id.btn_add_bg);
        btnAddBg.setOnClickListener(new OnClickListener() {

            @Override
            public void onClick(View v) {
                Intent gallery = new Intent(ACTION_PICK, EXTERNAL_CONTENT_URI);
                startActivityForResult(gallery, GALLERY_REQUEST_BG);
            }
        });

        btnCapture = (Button) rootView.findViewById(R.id.btn_capture);
        btnCapture.setOnClickListener(new OnClickListener() {

            @Override
            public void onClick(View v) {
                Bitmap bitmap = createCaptureBitmap();

                String savePath = Environment.getExternalStorageDirectory().toString() + "/capture/";

                saveBitmap(savePath, SAVED_CAPTURE_IMAGE_NAME, bitmap);

                Toast.makeText(getActivity(), "save at " + savePath + SAVED_CAPTURE_IMAGE_NAME, Toast.LENGTH_LONG)
                        .show();
            }
        });
        return rootView;
    }

    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent data) {

        switch (requestCode) {
        case GALLERY_REQUEST: {
            if (resultCode == Activity.RESULT_OK && data != null) {
                String imageUri = data.getData().toString();

                GestureTransformableImageView iv = createGestureImageView();

                if (iv != null) {
                    loadImagePicasso(imageUri, iv);
                }

                RelativeLayout rlGesture = (RelativeLayout) getView().findViewById(R.id.rl_gesture);
                // rlGesture.addView(iv);
                RelativeLayout.LayoutParams param = new RelativeLayout.LayoutParams(WC, WC);
                param.addRule(RelativeLayout.CENTER_IN_PARENT, 1);
                rlGesture.addView(iv, param);

                imageViewList.add(iv);

            } else {
                super.onActivityResult(requestCode, resultCode, data);
            }
            break;
        }
        case GALLERY_REQUEST_BG: {
            if (resultCode == Activity.RESULT_OK && data != null) {
                String imageUri = data.getData().toString();

                ImageView v = (ImageView) getView().findViewById(R.id.iv_background);

                try {
                    InputStream inputStream = getActivity().getContentResolver().openInputStream(Uri.parse(imageUri));
                    Drawable drawable = Drawable.createFromStream(inputStream, imageUri);
                    v.setImageDrawable(drawable);
                } catch (FileNotFoundException e) {
                }
            }
            break;
        }
        default:
            break;
        }
    }

    private GestureTransformableImageView createGestureImageView() {

        // GestureTransformableImageView gestureImageView = new GestureTransformableImageView(getActivity(),
        //         GestureTransformableImageView.GESTURE_DRAGGABLE | GestureTransformableImageView.GESTURE_ROTATABLE
        //                 | GestureTransformableImageView.GESTURE_SCALABLE);
        GestureTransformableImageView gestureImageView = new GestureTransformableImageView(getActivity(),
                GestureTransformableImageView.GESTURE_ROTATABLE);
        LayoutParams params = new LayoutParams(IMAGE_VIEW_WIDTH, IMAGE_VIEW_HEIGHT);
        gestureImageView.setLayoutParams(params);
        gestureImageView.setScaleType(ScaleType.CENTER_CROP);
        return gestureImageView;
    }

    private void loadImagePicasso(String imagePath, ImageView imageView) {
        Picasso.with(getActivity()).load(imagePath).resize(LOAD_IMAGE_RESIZE_WIDTH, LOAD_IMAGE_RESIZE_HEIGHT)
                .centerCrop().into(imageView);
    }

    private Bitmap createCaptureBitmap() {
        View v = getView().findViewById(R.id.rl_gesture);
        v.setDrawingCacheEnabled(true);
        Bitmap bitmap = Bitmap.createBitmap(v.getDrawingCache());
        v.setDrawingCacheEnabled(false);
        return bitmap;
    }

    public static boolean saveBitmap(String dirPath, String fileName, Bitmap data) {

        // fileパス
        File dirPathFile = new File(dirPath);
        // ファイル名
        String fileStr = dirPath + "/" + fileName;
        // file(パス+ファイル名)
        File filePath = new File(dirPathFile, fileName);

        // フォルダ生成
        try {
            /* ディレクトリチェック 無かったら作成 */
            if (!dirPathFile.isDirectory()) {
                if (!dirPathFile.mkdirs()) {
                }
            }
        } catch (SecurityException e) {
            e.printStackTrace();
        }

        // データ生成
        FileOutputStream fos = null;
        try {

            if (existsFile(fileStr)) {
                filePath.delete();
                filePath.createNewFile();
            } else {
                filePath.createNewFile();
            }

            fos = new FileOutputStream(fileStr);

            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            if (data != null) {
                data.compress(CompressFormat.JPEG, 100, bos);
                fos.write(bos.toByteArray());
            } else {
                return false;
            }

        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            if (fos != null) {
                try {
                    fos.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
        return true;
    }

    public static boolean existsFile(String filePath) {
        return (new File(filePath)).exists();
    }
}
