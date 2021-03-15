package com.dji.sdk.autopilot.internal.view;

import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.Button;
import android.widget.RelativeLayout;


import androidx.annotation.NonNull;

import com.dji.sdk.autopilot.R;
import com.dji.sdk.autopilot.demo.flightcontroller.VirtualStickView;


public class FristPageView extends RelativeLayout{

    Button flight_btn;

    public FristPageView(Context context) {
        super(context);
        init(context);
    }
    private void init(Context context) {
        LayoutInflater layoutInflater = (LayoutInflater) context.getSystemService(Service.LAYOUT_INFLATER_SERVICE);
        layoutInflater.inflate(R.layout.__first_page, this, true);

        flight_btn = (Button) findViewById(R.id.flight_button);

        flight_btn.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                Intent intent = new Intent(v.getContext(), FlightPageView.class);
                v.getContext().startActivity(intent);
            }
        });
    }

    @Override
    public int getHorizontalFadingEdgeLength() {
        return super.getHorizontalFadingEdgeLength();
    }

    @Override
    protected void onAttachedToWindow() {
        super.onAttachedToWindow();
    }

    @Override
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();
    }

}
