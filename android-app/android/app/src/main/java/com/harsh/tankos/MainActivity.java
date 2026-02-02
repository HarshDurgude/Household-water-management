package com.harsh.tankos;

import android.os.Bundle;
import android.view.View;
import android.webkit.WebView;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;
import com.getcapacitor.BridgeActivity;

public class MainActivity extends BridgeActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        WebView webView = this.bridge.getWebView();

        // 🔴 THIS IS THE ANDROID 15 FIX
        ViewCompat.setOnApplyWindowInsetsListener(webView, (v, insets) -> {
            int top = insets.getInsets(WindowInsetsCompat.Type.statusBars()).top;
            int bottom = insets.getInsets(WindowInsetsCompat.Type.navigationBars()).bottom;

            v.setPadding(0, top, 0, bottom);
            return insets;
        });
    }
}