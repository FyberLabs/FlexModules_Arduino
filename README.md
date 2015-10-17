# Flex Modules Arduino library repository

For our Flex Module boards, we will provide drivers.  We may provide links or forked versions of other open source drivers if they are applicable to our boards.

However, we do hope to develop our own custom drivers as much as possible.

We welcome volunteers and help developing drivers.  If you develop a partial driver that we can use, we will send you a free Flex Module to test with.  Contact us at <github@fyberlabs.com> for more information.

We will release our drivers under the BSD license.  Our coding style is similar to more recent Adafruit Arduino drivers.

We will utilize typedef enums instead of #defines to limit global namespace pollution.  Register addresses and values should be separate, but named similar typedefs.  The names should relate to function as well as the methods they interact with.  The typedefs can be placed in global space or public of the related classes declared.  Any state or combined registry fields should be stored in private variables obviously.

Please reference in comments any code you utilize (and only utilize license compatible code) in by author/email, url, and license.

If you have any questions, please file an issue or contact us directly at <github@fyberlabs.com>.  Thank you!
