package org.firstinspires.ftc.teamcode.AutoPlay;

import java.util.ArrayList;
import java.util.Arrays;

public class Board {
    public enum PIXEL_TYPE {
        NONE("N"),
        WHITE("W"),
        YELLOW("Y"),
        PURPLE("P"),
        GREEN("G");
        final String type;
        PIXEL_TYPE(String type){
            this.type = type;
        }
        @Override
        public String toString(){
            return type;
        }
    }

    public static class VALID_MOSAICS {
        public static final PIXEL_TYPE[] GREEN = {PIXEL_TYPE.GREEN,PIXEL_TYPE.GREEN,PIXEL_TYPE.GREEN};
        public static final PIXEL_TYPE[] PURPLE = {PIXEL_TYPE.PURPLE,PIXEL_TYPE.PURPLE,PIXEL_TYPE.PURPLE};
        public static final PIXEL_TYPE[] YELLOW = {PIXEL_TYPE.YELLOW,PIXEL_TYPE.YELLOW,PIXEL_TYPE.YELLOW};
        public static final PIXEL_TYPE[] MIXED = {PIXEL_TYPE.GREEN,PIXEL_TYPE.PURPLE,PIXEL_TYPE.YELLOW};

        public static ArrayList<PIXEL_TYPE[]> getAsList(){
            ArrayList<PIXEL_TYPE[]> list = new ArrayList<>();
            list.add(GREEN);
            list.add(PURPLE);
            list.add(YELLOW);
            list.add(MIXED);
            return list;
        }
    }

    public static final int TOTAL_ROWS = 11;
    public static final int LONG_ROW_LENGTH = 7;
    public static final int POINTS_PER_PIXEL = 3;
    public static final int POINTS_PER_MOSAIC = 10;
    public static final int POINTS_PER_SET_LINE = 10;
    public static final int[] SET_LINES = {2, 5, 8};
    public static int score = 0;

    public static class Pixel {
        public int index;
        public int x;
        public int y;
        public PIXEL_TYPE type = PIXEL_TYPE.NONE;
        public int rowLength;
        public ArrayList<Pixel> neighbors = new ArrayList<>();
        public boolean inMosaic = false;

        public Pixel(int index, int x, int y, int rowLength){
            this.index = index;
            this.x = x;
            this.y = y;
            this.rowLength = rowLength;
        }

        public boolean isMosaic(){
            // Don't bother if this pixel is already in a mosaic, not populated, or not a colored pixel.
            if (inMosaic || type == PIXEL_TYPE.NONE || type == PIXEL_TYPE.WHITE)
            {
                return false;
            }

            ArrayList<Pixel> coloredNeighbors = new ArrayList<>();
            for (int i = 0; i < neighbors.size(); i++)
            {
                if (neighbors.get(i).type != PIXEL_TYPE.NONE && neighbors.get(i).type != PIXEL_TYPE.WHITE)
                {
                    coloredNeighbors.add(neighbors.get(i));
                }
            }

            // If there are more or less than two colored neighbors, then this pixel is not part of a mosaic.
            if (coloredNeighbors.size() != 2)
            {
                return false;
            }

            // Start tracking the colors forming this mosaic.
            ArrayList<String> colors = new ArrayList<>();
            colors.add(type.toString());

            // If the 2 colored neighbors also only have two colored neighbors, then this may be a mosaic.
            for (int i = 0; i < 2; i++)
            {
                Pixel neighbor = coloredNeighbors.get(i);

                ArrayList<Pixel> neighborsColoredNeighbors1 = new ArrayList<>();
                for (int j = 0; j < neighbor.neighbors.size(); j++)
                {
                    if (neighbor.neighbors.get(j).type != PIXEL_TYPE.NONE && neighbor.neighbors.get(j).type != PIXEL_TYPE.WHITE) {
                        neighborsColoredNeighbors1.add(neighbor.neighbors.get(j));
                    }
                }

                if(neighborsColoredNeighbors1.size() != 2){
                    return false;
                }

                ArrayList<Pixel> neighborsColoredNeighbors = new ArrayList<>();
                for (int j = 0; j < neighbor.neighbors.size(); j++)
                {
                    if (neighbor.neighbors.get(j).type != PIXEL_TYPE.NONE && neighbor.neighbors.get(j).type != PIXEL_TYPE.WHITE &&
                            (neighbor.neighbors.get(j).neighbors.contains(this)||neighbor.neighbors.get(j)==this)) {
                        neighborsColoredNeighbors.add(neighbor.neighbors.get(j));
                    }
                }

                if (neighborsColoredNeighbors.size() != 2)
                {
                    return false;
                }

                colors.add(neighbor.type.toString());
            }

            // Verify that the color combination forms a valid mosaic.
            String[] colorsArray = new String[colors.size()];
            colorsArray = colors.toArray(colorsArray);
            Arrays.sort(colorsArray);

            for (PIXEL_TYPE[] mosaicType : VALID_MOSAICS.getAsList())
            {
                String colorString = colorsArray[0]+colorsArray[1]+colorsArray[2];
                String mosaicString = mosaicType[0].toString()+mosaicType[1].toString()+mosaicType[2].toString();
                if (colorString.equals(mosaicString))
                {
                    // Mark all pixels as in a mosaic before proceeding.
                    inMosaic = true;
                    for (int i = 0; i < coloredNeighbors.size(); i++)
                    {
                        coloredNeighbors.get(i).inMosaic = true;
                    }

                    return true;
                }
            }

            return false;
        }
    }

    public static class Backdrop {

        public ArrayList<Pixel> pixels = new ArrayList<>();

        public Backdrop()
        {
            for (int i = 0; i < TOTAL_ROWS; i++)
            {
                int rowLength = LONG_ROW_LENGTH - ((i + 1) % 2);

                for (int j = 0; j < rowLength; j++)
                {
                    pixels.add(new Pixel(pixels.size(), j, i, rowLength));
                }
            }

            // Set the neighbors for each pixel.
            for (int i = 0; i < pixels.size(); i++)
            {
                Pixel pixel = pixels.get(i);

                // Left
                if (pixel.x != 0)
                {
                    pixel.neighbors.add(pixels.get(i - 1));
                }

                // Right
                if (pixel.x != (pixel.rowLength - 1))
                {
                    pixel.neighbors.add(pixels.get(i + 1));
                }

                // Bottom
                if (pixel.y != 0)
                {
                    if (pixel.rowLength == LONG_ROW_LENGTH)
                    {
                        if (pixel.x != 0)
                        {
                            pixel.neighbors.add(pixels.get(i - LONG_ROW_LENGTH));
                        }

                        if (pixel.x != (pixel.rowLength - 1))
                        {
                            pixel.neighbors.add(pixels.get(i - (pixel.rowLength - 1)));
                        }
                    }
                    else
                    {
                        pixel.neighbors.add(pixels.get(i - (LONG_ROW_LENGTH - 1)));
                        pixel.neighbors.add(pixels.get(i - LONG_ROW_LENGTH));
                    }
                }

                // Top
                if (pixel.y != (TOTAL_ROWS - 1))
                {
                    if (pixel.rowLength == LONG_ROW_LENGTH)
                    {
                        if (pixel.x != 0)
                        {
                            pixel.neighbors.add(pixels.get(i + (LONG_ROW_LENGTH - 1)));
                        }

                        if (pixel.x != (LONG_ROW_LENGTH - 1))
                        {
                            pixel.neighbors.add(pixels.get(i + LONG_ROW_LENGTH));
                        }
                    }
                    else
                    {
                        pixel.neighbors.add(pixels.get(i + (LONG_ROW_LENGTH - 1)));
                        pixel.neighbors.add(pixels.get(i + LONG_ROW_LENGTH));
                    }
                }
            }
        }

        public int score(){
            return score(pixels);
        }

        public int score(ArrayList<Pixel> board)
        {
            // Clear the inMosaic flags for the pixels.
            for (int i = 0; i < board.size(); i++)
            {
                board.get(i).inMosaic = false;
            }

            int totalPixels = 0;
            int pixelScore = 0;

            int highestRow = -1;
            int totalSetLines = 0;
            int setLineScore = 0;

            int mosaicCount = 0;
            int mosaicScore = 0;

            for (int i = 0; i < board.size(); i++)
            {
                if (board.get(i).type != PIXEL_TYPE.NONE)
                {
                    // Score each pixels individually.
                    totalPixels++;

                    // Update the highest row if necessary.
                    if (board.get(i).y > highestRow)
                    {
                        highestRow = board.get(i).y;
                    }

                    // Increment the mosaic counter if necessary.
                    if (board.get(i).isMosaic())
                    {
                        mosaicCount++;
                    }
                }
            }

            // Calculate the score.
            pixelScore = totalPixels * POINTS_PER_PIXEL;
            mosaicScore = mosaicCount * POINTS_PER_MOSAIC;

            for (int i = 0; i < SET_LINES.length; i++)
            {
                if (highestRow >= SET_LINES[i])
                {
                    totalSetLines++;
                }
            }

            setLineScore = totalSetLines * POINTS_PER_SET_LINE;

            System.out.println("Set Line: " + setLineScore);
            System.out.println("Pixel Score: " + pixelScore);
            System.out.println("Mosaic Score:" + mosaicScore);

            return pixelScore + setLineScore + mosaicScore;
        }

        public Action[] getPossibleActions(){
            ArrayList<Action> actions = new ArrayList<>();
            for (int y = 0; y < 12; y++){
                for (int x = 0; x < 7; x++){
                    Pixel pixel = findPixel(x,y);
                    if(pixel == null)
                        continue;
                    if(pixel.type == PIXEL_TYPE.NONE){
                        if(y == 0 ||
                                (x == 0 && findPixel(x,Math.max(y-1, 0)).type != PIXEL_TYPE.NONE && y%2 !=0) ||
                                (x == 0 && findPixel(x,Math.max(y-1, 0)).type != PIXEL_TYPE.NONE && y%2 == 0 &&
                                        findPixel(x+1,Math.max(y-1, 0)).type != PIXEL_TYPE.NONE) ||
                                (x == 6 && findPixel(Math.max(x-1, 0),Math.max(y-1, 0)).type != PIXEL_TYPE.NONE) ||
                                (findPixel(y % 2 != 0 ? Math.max(x-1, 0) : x+1,Math.max(y-1, 0)).type != PIXEL_TYPE.NONE
                                        && findPixel(x,Math.max(y-1, 0)).type != PIXEL_TYPE.NONE)){
                            actions.add(new Action(PIXEL_TYPE.GREEN, x,y));
                            actions.add(new Action(PIXEL_TYPE.PURPLE, x,y));
                            actions.add(new Action(PIXEL_TYPE.YELLOW, x,y));
                            actions.add(new Action(PIXEL_TYPE.WHITE, x,y));
                        }
                    }
                }
            }
            return actions.toArray(new Action[0]);
        }

        public Pixel findPixel(int x, int y){
            for(Pixel pixel : pixels){
                if(pixel.x == x && pixel.y == y){
                    return pixel;
                }
            }
            return null;
        }

        public Action[] getBestActions(){
            int currentScore = score();
            System.out.println(currentScore);
            Board.Action[] possibleActions = getPossibleActions();
            int maxScore = currentScore;
            Board.Action[] maxScoreActions = null;
            for(Board.Action action : possibleActions){
                Board.Pixel pixel = findPixel(action.x, action.y);
                Board.PIXEL_TYPE type = pixel.type;
                pixel.type = action.type;
                for(Board.Action action1 : getPossibleActions()){
                    Board.Pixel pixel1 = findPixel(action1.x, action1.y);
                    Board.PIXEL_TYPE type1 = pixel1.type;
                    pixel1.type = action1.type;
                    for (Board.Action action2 : getPossibleActions()){
                        Board.Pixel pixel2 = findPixel(action2.x, action2.y);
                        Board.PIXEL_TYPE type2 = pixel2.type;
                        pixel2.type = action2.type;
                        int score = score();
                        if(score > maxScore){
                            maxScore = score;
                            maxScoreActions = new Board.Action[] {action, action1, action2};
                        }
                        findPixel(action2.x, action2.y).type = type2;
                    }
                    findPixel(action1.x, action1.y).type = type1;
                }
                findPixel(action.x, action.y).type = type;
            }
            score = maxScore;
            return maxScoreActions;
        }
    }

    public static class Action{
        public PIXEL_TYPE type;
        public int x;
        public int y;

        public Action(PIXEL_TYPE type, int x, int y){
            this.type = type;
            this.x = x;
            this.y = y;
        }

        @Override
        public String toString(){
            return "("+type + " " + x + ", " + y +")\n";
        }
    }

}
