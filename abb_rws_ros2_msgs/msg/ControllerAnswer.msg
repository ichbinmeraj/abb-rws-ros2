# Generated from the abb-rws-daemon JSON Schema by generate_msgs.py. Do not edit.
#
# What the controller itself answered when it failed or refused a request, so a consumer can show
# the controller's own code and words next to the daemon's `code`. Present only when the
# controller's answer carried at least one of the three; each is null when it carried that one not.
# Never carries credentials or request headers: the message is the controller's error text with
# anything credential-like masked, on one line, at most CONTROLLER_MESSAGE_MAX characters. Added in
# schema 0.5.

# The HTTP status the controller answered with (e.g. 403); null when no answer reached the daemon.
# Valid only when http_status_known is true.
int64 http_status
bool http_status_known

# The controller's own status code from its error body (e.g. -1073445862); null when the body had
# none.
# Valid only when code_known is true.
int64 code
bool code_known

# The controller's own message from its error body; null when the body had none.
# Valid only when message_known is true.
string message
bool message_known
