function playback( pos )
%PLAYBACK Summary of this function goes here
%   Detailed explanation goes here


csvwrite('pos_output.csv',pos);

system('"./playback"')

end

