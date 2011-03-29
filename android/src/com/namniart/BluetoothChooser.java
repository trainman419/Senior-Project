/**
 * 
 */
package com.namniart;

import java.util.ArrayList;

import android.app.ListActivity;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.os.Bundle;
import android.os.Parcelable;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.ListView;

/**
 * An activity to choose a bluetooth device. <br/>
 * <br/>
 * It expects an ArrayList&lt;Parcelable&gt; of BluetoohDevices attached to the Intent as
 * an extra, with index "devices". <br/>
 * <br/>
 * Presents the list of bluetooth devices in large font to the user, and returns the index
 * of the chosen device. <br/>
 * 
 * 
 * @author Austin Hendrix
 *
 */
public class BluetoothChooser extends ListActivity {
		
    @Override
    public void onCreate(Bundle savedInstanceState) {
    	super.onCreate(savedInstanceState);
        setContentView(R.layout.bluetooth);
        ArrayAdapter<String> adapter = new ArrayAdapter<String>(this, R.layout.bluetooth_row);
        BluetoothDevice dev;
        
    	Intent i = getIntent();
    	ArrayList<Parcelable> list = i.getParcelableArrayListExtra("devices");
    	for( Parcelable p : list ) {
    		dev = (BluetoothDevice) p;
    		adapter.add(dev.getName());
    	}
    	setListAdapter(adapter);
    }
    
    @Override
    protected void onListItemClick(ListView l, View v, int position, long id) {
        super.onListItemClick(l, v, position, id);
        Intent i = new Intent();
        i.putExtra("index", position);
        
		setResult(RESULT_OK, i);
		finish();
    }


}
