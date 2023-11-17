% Copyright 2021-2022 The MathWorks, Inc.
 
classdef MyCustomAprilTagPatternDetector < vision.calibration.PatternDetector
    
    %----------------------------------------------------------------------
    properties(Constant)
        % Name: Provide a name for the pattern detector
        Name = 'AprilTag';
    end
    
    %----------------------------------------------------------------------
    % UI components for the properties panel
    %----------------------------------------------------------------------
    properties (Access = private)
        TagSizeEditBox;
        
        TagArrangementRowEditBox;
        
        TagArrangementColEditBox;
        
        UnitsPopup;
        
        AvailableUnits = {'millimeters','centimeters','inches'};
        
        TagFamilyPopup;
        
        AvailableTagFamilies = {'tag36h11','tag16h5','tag25h9',...
            'tagCircle21h7','tagCircle49h12','tagCustom48h12',...
            'tagStandard41h12','tagStandard52h13'};
        
        PanelGridLayout

        PropertiesGridLayout

        ThumbnailGridLayout
    end
    
    %----------------------------------------------------------------------
    properties
        % WorldUnits: Specify a unit measure for the world points
        WorldUnits = 'millimeters';
        
        SelectedTagFamily = 'tag36h11';
         
        TagSize = 16;
        
        TagArrangementRow = int64(5);
        
        TagArrangementCol = int64(8);
        
        BoardSize;
    
        % Panel: Store the uipanel handle provided using propertiesPanel method
        Panel;
    end
    
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        % Implement the detectPatternPoints method to process the
        % calibration image files to detect pattern keypoints to be used for
        % calibration. The detection should produce an M-by-2-numImages
        % array of imagePoints and a logical array, imagesUsed of the same
        % size as numImages indicating the detection success.
        %
        % For more help,
        %   >> doc vision.calibration.PatternDetector.detectPatternPoints
        function [imagePoints, imagesUsed] = detectPatternPoints(this, imageFileNames, varargin)           
             
            tagArrangement = [double(this.TagArrangementRow), double(this.TagArrangementCol)];
            imagesCalib = imageDatastore(imageFileNames);
            [imagePoints, boardSize, imagesUsed] = helperDetectAprilTagCorners(...
                imagesCalib, tagArrangement, this.SelectedTagFamily);
            
            this.BoardSize = boardSize;
        end
         
        %------------------------------------------------------------------
        % Implement the generateWorldPoints method to generate x-y world
        % coordinates corresponding to the planar pattern keypoints. The
        % world frame is assumed to be attached to the pattern with the XY
        % plane associated with the pattern plane.
        %
        % For more help,
        %   >> doc vision.calibration.PatternDetector.generateWorldPoints
        %------------------------------------------------------------------
        function worldPoints = generateWorldPoints(this, varargin)
            
            worldPoints = generateCheckerboardPoints(this.BoardSize, this.TagSize);
        end      
    end
    
    %----------------------------------------------------------------------
    methods
         
        %------------------------------------------------------------------
        % Optional: This method will be used to render the origin, X-axis
        % and Y-axis labels in the calibration images displayed in the calibrator
        % apps. This method is invoked after the pattern keypoints have
        % been successfully detected in the images.
        %
        % For more help,
        %   >> doc vision.calibration.PatternDetector.drawImageAxesLabels
        function [originLabel, xLabel, yLabel] = drawImageAxesLabels(this, imagePoints)
            
            [originLabel, xLabel, yLabel] = helperDrawImageAxesLabels(this.BoardSize, imagePoints);
        end
        
        %------------------------------------------------------------------
        % Optional: This method will be used to populate the Properties
        % panel in the Image and Pattern Properties dialog. This can be
        % used to provide UI elements to gather information needed for the
        % execution of the detector.
        %
        % For more help,
        %   >> doc vision.calibration.PatternDetector.propertiesPanel
        function propertiesPanel(this, panel)
            this.Panel = panel;
             
            % Example code used to configure the properties panel. This call
            % sets up an UI text element in the InfoLabelBox property.
            configureUIComponents(this);
            
            % Initialize property values
            initializePropertyValues(this);
        end
        
    end
    
    %----------------------------------------------------------------------
    methods (Access = private)
        %--------------------------------------------------------------
        %  Configure UI components using the following grid layout
        %--------------------------------------------------------------
        %  ---------------------------------
        % | Properties Panel                |
        % |---------------------------------|
        % |        |               |        |  <--- Row 1
        % |------------------------|        |
        % |        |       |       |        |  <--- Row 2
        % |------------------------|        |
        % |      |     |     |     |        |  <--- Row 3
        %  ---------------------------------
        function configureUIComponents(this)
            
            % Create a grid layout with two columns for properties and
            % thumbnail.
            this.PanelGridLayout = uigridlayout(this.Panel);
            this.PanelGridLayout.RowHeight = {'fit'};
            this.PanelGridLayout.ColumnWidth = {'3x','1x'};

            % Create a sub-grid layout inside the first column with three
            % rows.
            this.PropertiesGridLayout = uigridlayout(this.PanelGridLayout, 'Padding', 2);
            this.PropertiesGridLayout.RowHeight = {'fit', 'fit', 'fit'};
            this.PropertiesGridLayout.ColumnWidth = {'fit'};

            % Configure the UI components in the three rows.
            configureTagFamilyUIComponents(this);
            configureTagSizeUIComponents(this);
            configureTagDimsUIComponents(this);

            % Configure the UI component for pattern thumbnail image .
            this.ThumbnailGridLayout = uigridlayout(this.PanelGridLayout, 'Padding', 2);
            this.ThumbnailGridLayout.RowHeight = {'fit'};
            this.ThumbnailGridLayout.ColumnWidth = {'fit'};
            configurePatternThumbnailImage(this);
        end

        %------------------------------------------------------------------
        % UI components for Tag Family
        %------------------------------------------------------------------
        function configureTagFamilyUIComponents(this)

            tagFamilyGridLayout = uigridlayout(this.PropertiesGridLayout, 'Padding', 2);
            tagFamilyGridLayout.RowHeight = {'fit'};
            tagFamilyGridLayout.ColumnWidth = {'fit','fit'};

            % Label            
            uilabel('Parent', tagFamilyGridLayout,...
                'HorizontalAlignment', 'Left', 'Text', 'Tag Family:');
            
            % Tag family popup
            initTagFamily = 'tag36h11';
            this.TagFamilyPopup = uidropdown('Parent', tagFamilyGridLayout,...
                'Items', this.AvailableTagFamilies,...
                'ValueChangedFcn', @(~, ~) doTagFamilyChanged(this),...
                'Value', initTagFamily,...
                'Tooltip', 'Choose a Tag Family');
        end

        %------------------------------------------------------------------
        % UI components for Tag Size
        %------------------------------------------------------------------
        function configureTagSizeUIComponents(this)
       
            tagSizeGridLayout = uigridlayout(this.PropertiesGridLayout, 'Padding', 2);
            tagSizeGridLayout.RowHeight = {'fit'};
            tagSizeGridLayout.ColumnWidth = {'fit','fit', 'fit'};

            % Label 
            uilabel('Parent', tagSizeGridLayout,...
                'HorizontalAlignment', 'Left', 'Text', 'Square Size:');
            
            % Editbox
            initSquareSize = 16;
            this.TagSizeEditBox = uieditfield('Parent', tagSizeGridLayout, ...
                'Editable','on', ...
                'HorizontalAlignment', 'left', ...:
                'BackgroundColor',[1 1 1],...
                'Value', num2str(initSquareSize), ...
                'ValueChangedFcn', @(~, ~) doTagSizeChanged(this), ...
                'Tooltip', 'Square Size');
            
            % Units selection dropdown
            initUnits = 'millimeters';
            this.UnitsPopup = uidropdown('Parent', tagSizeGridLayout,...
                'Items', this.AvailableUnits,...
                'ValueChangedFcn', @(~, ~) doUnitsChanged(this),...
                'Value', initUnits,...
                'Tooltip', 'Unit of Size');
        end

        %------------------------------------------------------------------
        % UI components for Tag Dimensions 
        %------------------------------------------------------------------
        function configureTagDimsUIComponents(this)

            tagDimsGridLayout = uigridlayout(this.PropertiesGridLayout, 'Padding', 2);
            tagDimsGridLayout.RowHeight = {'fit'};
            tagDimsGridLayout.ColumnWidth = {'fit', 55, 'fit', 55};

            % Rows Label 
            uilabel('Parent',tagDimsGridLayout,...
                'HorizontalAlignment', 'Left', 'Text', 'Number of Rows:');
            
            % Rows Editbox
            initTagArrangementRow = int64(5);
            this.TagArrangementRowEditBox = uieditfield('Parent', tagDimsGridLayout, ...
                'Editable','on', ...
                'HorizontalAlignment', 'left', ...:
                'BackgroundColor',[1 1 1],...
                'Value', num2str(initTagArrangementRow), ...
                'ValueChangedFcn', @(~, ~) doTagArrangementRowChanged(this), ...
                'Tooltip', 'Number of Rows');
            
            % Columns Label 
            uilabel('Parent',tagDimsGridLayout,...
                'HorizontalAlignment', 'Left', 'Text', 'Number of Columns:');
            
            % Columns Editbox
            initTagArrangementCol = int64(8);
            this.TagArrangementColEditBox = uieditfield('Parent', tagDimsGridLayout, ...
                'Editable','on', ...
                'HorizontalAlignment', 'left', ...:
                'BackgroundColor',[1 1 1],...
                'Value', num2str(initTagArrangementCol), ...
                'ValueChangedFcn', @(~, ~) doTagArrangementColChanged(this), ...
                'Tooltip', 'Number of Columns');
        end

        %------------------------------------------------------------------
        % UI components for Thumbnail 
        %------------------------------------------------------------------
        function configurePatternThumbnailImage(this)

            % Display visual guide.
            visualGuideFile = 'aprilTagThumbnail.png';
            vision.internal.calibration.tool.displayPatternThumbnail( ...
                this.ThumbnailGridLayout, visualGuideFile);
        end

        %------------------------------------------------------------------
        % Initialize default values of UI components
        %------------------------------------------------------------------
        function initializePropertyValues(this)
            
            % Tag family
            this.SelectedTagFamily = get(this.TagFamilyPopup,'value');
            
            % TagSize
            this.TagSize = str2double(get(this.TagSizeEditBox,'value'));
            
            if this.TagSize <= 0 || isnan(this.TagSize) || ~isfloat(this.TagSize)
                errordlg('Invalid tag size');
            end
            
            % Units
            this.WorldUnits = get(this.UnitsPopup,'value');
            
            % TagArrangementRow
            this.TagArrangementRow = int64(str2double(get(this.TagArrangementRowEditBox,'value')));
            
            if this.TagArrangementRow <= 0 || isnan(this.TagArrangementRow) || ~isinteger(this.TagArrangementRow)
                errordlg('Invalid number of tag rows');
            end
            
            % TagArrangementCol
            this.TagArrangementCol = int64(str2double(get(this.TagArrangementColEditBox,'value')));
            
            if this.TagArrangementCol <= 0 || isnan(this.TagArrangementCol) || ~isinteger(this.TagArrangementCol)
                errordlg('Invalid number of tag columns');
            end
        end
    end
        
    %----------------------------------------------------------------------
    % Callbacks for UI components to update corresponding properties
    %----------------------------------------------------------------------
    methods
        
        %------------------------------------------------------------------
        function doTagFamilyChanged(this)
            this.SelectedTagFamily = get(this.TagFamilyPopup,'value');
        end
        
        %------------------------------------------------------------------
        function doTagSizeChanged(this)
            this.TagSize = str2double(get(this.TagSizeEditBox,'value'));
            
            if this.TagSize <= 0 || isnan(this.TagSize) || ~isfloat(this.TagSize)
                errordlg('Invalid tag size');
            end
        end
        
        %------------------------------------------------------------------
        function doUnitsChanged(this)
            this.WorldUnits = get(this.UnitsPopup,'value');
        end
        
        %------------------------------------------------------------------
        function doTagArrangementRowChanged(this)
            this.TagArrangementRow = str2double(get(this.TagArrangementRowEditBox,'value'));
            
            if this.TagArrangementRow <= 0 || isnan(this.TagArrangementRow) || ~isinteger(this.TagArrangementRow)
                errordlg('Invalid number of tag rows');
            end
        end
        
        %------------------------------------------------------------------
        function doTagArrangementColChanged(this)
            this.TagArrangementCol = str2double(get(this.TagArrangementColEditBox,'value'));
            
            if this.TagArrangementCol <= 0 || isnan(this.TagArrangementCol) || ~isinteger(this.TagArrangementCol)
                errordlg('Invalid number of tag columns');
            end
        end
    end
end