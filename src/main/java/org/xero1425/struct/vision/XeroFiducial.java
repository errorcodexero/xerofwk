package org.xero1425.struct.vision;

import org.xero1425.base.LimelightHelpers.LimelightTarget_Fiducial;

import edu.wpi.first.util.struct.StructSerializable;

public class XeroFiducial implements StructSerializable {

    public final double id;
    public final double area;
    public final double x;
    public final double y;
    
    public XeroFiducial(
        double id,
        double area,
        double x,
        double y
    ) {
        this.id = id;
        this.area = area;
        this.x = x;
        this.y = y;
    }

    public XeroFiducial(LimelightTarget_Fiducial llFid) {
        this.id = llFid.fiducialID;
        this.area = llFid.ta;
        this.x = llFid.tx;
        this.y = llFid.ty;
    }

    public static XeroFiducial[] fromLimelightArray(LimelightTarget_Fiducial[] llArray) {
        XeroFiducial[] arr = new XeroFiducial[llArray.length];

        for (int i = 0; i < arr.length; i++) {
            arr[i] = new XeroFiducial(llArray[i]);
        }

        return arr;
    }

    public static final XeroFiducialStruct struct = new XeroFiducialStruct();
}